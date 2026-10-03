"""Hardware-in-the-loop simulator: the whole car with simulated hardware.

Runs the car's real firmware (compiled for the PC) behind a pseudo serial
port that the real serial bridge opens, moves a tank chassis from the
firmware's motor pins through the real AQ/MBC/WMC floor geometry, and feeds
the real drivers: stereo images (as usb_cam publishes them), or with
``cameras:=iphone`` the iPhone's Record3D USB stream (RGB, LiDAR depth, ARKit
pose, read by the real iPhone driver through ``fake_record3d``), the phone's
Sensor Logger HTTP pushes, NMEA from the GPS on a second pseudo port, the
ultrasonic echo the firmware times. It owns the clock: everything else runs
on /clock, so slow rendering slows simulated time instead of starving the
controllers.

Published for tests: ``sim/truth`` (PoseStamped, map frame) and
``sim/status`` (JSON: floor, collisions, bumper range, firmware and phone
link counters). ``sim/button`` (Bool) presses the push-to-talk button;
``sim/stop_phone`` (Bool) cuts the phone's WiFi; ``sim/unplug_bumper`` (Bool)
disconnects the ultrasonic sensor.
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, String
from tf2_ros import TransformBroadcaster

from .chassis import Chassis, ChassisParams
from guide_bridge.arkit import arkit_camera_pose

from .devices import Phone, PtyPort, Record3DServer, nmea_sentences, utc_at
from .emulators import SignReader, floor_clouds, nearest_person
from .firmware import Firmware, default_ino
from .render import CameraModel, Renderer
from .world import Box, Person, World

PHYSICS_DT = 0.001  # s: firmware loop and chassis step
CLOCK_DT = 0.005  # s between /clock messages


def _stamp(t: float):
    from builtin_interfaces.msg import Time as TimeMsg

    sec = int(t)
    return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def _load_graph(maps: Path):
    g = json.loads((maps / "route_graph.json").read_text())
    nodes = {n["id"]: n for n in g["nodes"]}
    dests = json.loads((maps / "destinations.json").read_text())
    return nodes, dests


class HilSim(Node):
    def __init__(self):
        super().__init__("hil_sim")
        P = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        root = Path(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()))
        self.maps = Path(P("maps_dir", str(root / "maps")))
        self.nodes, self.dests = _load_graph(self.maps)
        start = P("start", "entrance:aq_convocation_mall")
        n = self.nodes[start]
        self.world = World(self.maps, n["floor"])
        cameras = P("cameras", "stereo")
        self.mono, self.iphone = cameras == "mono", cameras == "iphone"
        self.camera_rate = float(P("camera_rate", 10.0))
        self.rtf = float(P("max_rtf", 1.0))
        self.emulate_floor = bool(P("emulate_floor_segmentation", True))
        self.emulate_people = bool(P("emulate_person_detection", True))
        self.emulate_signs = bool(P("emulate_sign_reader", True))
        link_dir = P("link_dir", "/tmp/guide_sim")
        # Mapping (phase 1): carry the stereo rig along the route to this destination
        # and back, as a person would, instead of driving the car.
        self.rig_destination = P("rig_destination", "")
        self.rig = None

        # Hardware ----------------------------------------------------------------
        cp = ChassisParams(understeer=float(P("understeer", 1.1)), speed_at_ref=float(P("speed_at_ref", 1.5)))
        self.chassis = Chassis(cp)
        yaw = P("start_yaw", float("nan"))
        if not math.isfinite(yaw):
            yaw = self.open_direction(n["x"], n["y"])
        self.chassis.place(n["x"], n["y"], yaw)
        ino = P("firmware", str(default_ino()))
        self.fw = Firmware(Path(ino), defines=("HAVE_ULTRASONIC",))
        self.motor_port = PtyPort(f"{link_dir}/motor")
        self.gps_port = PtyPort(f"{link_dir}/gps")
        self.phone = Phone(P("phone_url", "http://127.0.0.1:8765/data"))
        self.cam = CameraModel()
        self.renderer = Renderer(self.cam)
        if self.iphone:
            # iPhone wide camera, landscape, about 65 deg across, on the stereo pair's mount.
            # Its LiDAR depth is 256 x 192; ARKit's tracking drifts about 1 % of distance.
            self.iphone_cam = CameraModel(width=640, height=480, fx=500.0, fy=500.0, cx=320.0, cy=240.0, baseline=0.0, noise=2.0)
            self.iphone_renderer = Renderer(self.iphone_cam, seed=1)
            self.r3d = Record3DServer(f"{link_dir}/record3d.sock")
            self.ark = [0.0, 0.0, 0.0]  # ARKit's own estimate of base_link (its world starts at the car)
            self.ark_last = None
            self.ark_scale = float(P("arkit_scale_error", 0.01))
            self.ark_yaw_drift = float(P("arkit_yaw_drift", 0.0005))  # rad/s
            self.phone.enabled = False  # the phone is the camera now: no Sensor Logger
        self.button = False
        self.echo_m = math.inf

        # The world beyond the map: clutter and people on the start floor.
        self.populate(int(P("clutter", 6)), int(P("people", 3)), int(P("seed", 7)))
        doors: dict[str, list] = {}
        for key, d in self.dests.items():
            node = self.nodes.get(d["node"])
            if node is not None:
                doors.setdefault(d["floor"], []).append((key, node["x"], node["y"]))
        self.signs = SignReader(doors)
        if self.rig_destination:
            self.rig = self.plan_rig(start, self.rig_destination)

        # ROS ---------------------------------------------------------------------
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.img_pub = {s: self.create_publisher(Image, f"/stereo/{s}/image_raw", 5) for s in ("left", "right")}
        self.info_pub = {s: self.create_publisher(CameraInfo, f"/stereo/{s}/camera_info", 5) for s in ("left", "right")}
        self.person_pub = self.create_publisher(Float32, "/people/nearest", 10)
        self.sign_pub = self.create_publisher(String, "/signs", 10)
        self.floor_obs_pub = self.create_publisher(PointCloud2, "/floor/obstacles", 5)
        self.floor_free_pub = self.create_publisher(PointCloud2, "/floor/free", 5)
        self.truth_pub = self.create_publisher(PoseStamped, "/sim/truth", 10)
        self.status_pub = self.create_publisher(String, "/sim/status", 10)
        self.tf = TransformBroadcaster(self)
        self.rig_pub = self.create_publisher(String, "/sim/rig_node", 10)
        self.create_subscription(String, "/guide/localization_mode", self.on_mode, 10)
        self.create_subscription(Bool, "/sim/button", lambda m: setattr(self, "button", m.data), 10)
        self.create_subscription(Bool, "/sim/stop_phone", lambda m: setattr(self.phone, "enabled", not m.data), 10)
        self.bumper_connected = True
        self.create_subscription(Bool, "/sim/unplug_bumper", lambda m: setattr(self, "bumper_connected", not m.data), 10)
        self.t = 0.0
        self.get_logger().info(f"motor board on {self.motor_port.path}, GPS on {self.gps_port.path}, "
                               f"start {start} yaw {math.degrees(yaw):.0f} deg, {len(self.world.here.boxes)} boxes, "
                               f"{len(self.world.here.people)} people")

    # --- World setup -----------------------------------------------------------------
    def open_direction(self, x: float, y: float) -> float:
        angs = np.linspace(-math.pi, math.pi, 72, endpoint=False)
        h = self.world.here.raycast(x, y, angs, max_range=40.0, people=False)
        return float(angs[np.argmax(np.where(np.isfinite(h.distance), h.distance, 40.0))])

    def populate(self, clutter: int, people: int, seed: int):
        """Boxes beside the corridors and people walking them, away from the start."""
        rng = np.random.default_rng(seed)
        fw = self.world.here
        corridor = [n for n in self.nodes.values() if n["floor"] == fw.floor and n["type"] in ("corridor", "corridor_junction")]
        sx, sy = self.chassis.s.x, self.chassis.s.y
        far = [n for n in corridor if math.hypot(n["x"] - sx, n["y"] - sy) > 6.0]
        rng.shuffle(far)
        placed = 0
        for n in far:
            if placed >= clutter:
                break
            # Against a wall: step from the centreline towards the nearest wall, leave 1.2 m clear.
            angs = np.linspace(-math.pi, math.pi, 36, endpoint=False)
            h = fw.raycast(n["x"], n["y"], angs, max_range=6.0, people=False)
            i = int(np.argmin(h.distance))
            d = float(h.distance[i])
            far_side = float(np.max(np.where(np.isfinite(h.distance), h.distance, 6.0)))
            if not math.isfinite(d) or d + far_side < 2.6:
                continue  # narrow corridor: no room for clutter and the car
            bx, by = n["x"] + (d - 0.35) * math.cos(angs[i]), n["y"] + (d - 0.35) * math.sin(angs[i])
            fw.add_box(Box(bx, by, 0.3, 0.3))
            placed += 1
        rng.shuffle(corridor)
        for n in corridor[:people]:
            # Walk 8 m back and forth along the longest free direction.
            yaw = self.open_direction(n["x"], n["y"])
            a = (n["x"], n["y"])
            b = (n["x"] + 8 * math.cos(yaw), n["y"] + 8 * math.sin(yaw))
            fw.people.append(Person(a, b, speed=float(rng.uniform(0.8, 1.3)), phase=float(rng.uniform(0, 16))))

    def plan_rig(self, start: str, destination: str) -> dict:
        from sfu_map.route import RouteGraph

        rg = RouteGraph.load(self.maps / "route_graph.json")
        path, _ = rg.shortest_path(start, rg.resolve(destination))
        floor = self.nodes[start]["floor"]
        path = [n for n in path if self.nodes[n]["floor"] == floor]  # one floor's database
        # There and back (both directions in the map, so the car can relocalize either
        # way), then the first stretch again facing the original way to close the loop.
        route = path + path[-2::-1] + path[1:6]
        self.get_logger().info(f"mapping rig: {len(route)} nodes to {destination} and back")
        return {"route": route, "i": 1, "done": False}

    def rig_step(self, dt: float):
        """A person walking the rig along the route: 0.4 m/s, turning at up to 30 deg/s."""
        rig, s = self.rig, self.chassis.s
        if rig["done"]:
            s.v = s.w = 0.0
            return
        n = self.nodes[rig["route"][rig["i"]]]
        dx, dy = n["x"] - s.x, n["y"] - s.y
        dist = math.hypot(dx, dy)
        if dist < 0.15:
            self.rig_pub.publish(String(data=n["id"]))
            rig["i"] += 1
            if rig["i"] >= len(rig["route"]):
                rig["done"] = True
                self.get_logger().info("mapping rig: route done")
            return
        err = (math.atan2(dy, dx) - s.yaw + math.pi) % (2 * math.pi) - math.pi
        s.w = max(-0.5, min(0.5, 2.0 * err))
        s.v = 0.4 if abs(err) < 0.3 else 0.05
        s.v = min(s.v, dist / dt)
        s.yaw += s.w * dt
        s.x += s.v * math.cos(s.yaw) * dt
        s.y += s.v * math.sin(s.yaw) * dt

    def on_mode(self, msg: String):
        """The mission says which floor the car is on: after an elevator ride, move the car there."""
        try:
            floor = json.loads(msg.data).get("floor")
        except ValueError:
            return
        if not floor or floor == self.world.floor:
            return
        s = self.chassis.s
        lifts = [n for n in self.nodes.values() if n["type"] == "elevator" and n["floor"] == self.world.floor
                 and math.hypot(n["x"] - s.x, n["y"] - s.y) < 5.0]
        if not lifts:
            return  # not at an elevator (the mission repeats the mode; nothing to do)
        here = lifts[0]
        targets = [n for n in self.nodes.values() if n["type"] == "elevator" and n["floor"] == floor]
        if not targets:
            return
        there = min(targets, key=lambda n: math.hypot(n["x"] - here["x"], n["y"] - here["y"]))
        self.world.floor = floor
        self.chassis.place(there["x"], there["y"], self.open_direction(there["x"], there["y"]))
        self.get_logger().info(f"elevator: car now on {floor} at {there['id']}")

    # --- Simulation ------------------------------------------------------------------
    def camera_xy(self):
        s = self.chassis.s
        return self.renderer.camera_pose(s.x, s.y, s.yaw, "center" if self.iphone else "left")

    def ultrasonic(self) -> float:
        s = self.chassis.s
        bx, by = s.x + 0.375 * math.cos(s.yaw), s.y + 0.375 * math.sin(s.yaw)  # on the front bumper
        angs = s.yaw + np.radians([-7.5, 0.0, 7.5])
        h = self.world.here.raycast(bx, by, angs, max_range=4.0, t=self.t)
        return float(np.min(h.distance))

    def physics_step(self):
        t = self.t + PHYSICS_DT
        fw, s = self.fw, self.chassis.s
        self.fw.serial_in(self.motor_port.read())
        fw.set_input(self.chassis.wiring.button, not self.button)  # pressed pulls the pin to ground
        # HC-SR04: 58 us per cm; nothing within 4 m still gives a ~38 ms pulse; unplugged gives none.
        fw.set_echo_us(0 if not self.bumper_connected else (self.echo_m * 100 * 58 if self.echo_m < 4.0 else 38000))
        fw.run_until(t)
        self.motor_port.write(fw.serial_out())
        v_before = s.v
        if self.rig is not None:
            self.rig_step(PHYSICS_DT)
        else:
            speed, steer = self.chassis.targets(fw)
            moving = abs(speed) + abs(s.v) > 1e-4
            here = self.world.here
            self.chassis.step(PHYSICS_DT, speed, steer,
                              (lambda x, y, yaw: here.collides(self.chassis.footprint(x, y, yaw), t)) if moving else None)
        self.phone.update(t, s.yaw, s.w, (s.v - v_before) / PHYSICS_DT, outdoors=False)
        self.t = t

    def publish_iphone(self):
        s, c = self.chassis.s, self.iphone_cam
        img, depth = self.iphone_renderer.render(self.world.here, s.x, s.y, s.yaw, "center", t=self.t, with_depth=True)
        rgb = np.repeat(img[:, :, None], 3, axis=2)
        d = cv2.resize(depth, (256, 192), interpolation=cv2.INTER_NEAREST)
        d = (d + self.iphone_renderer.rng.normal(0, 1, d.shape) * (0.01 + 0.01 * d)).astype(np.float32)
        conf = np.where((d > 0) & (d < 5.0), 2, 0).astype(np.uint8)  # beyond about 5 m the LiDAR has nothing
        # ARKit: integrate the true motion with a scale error and a slow heading drift.
        if self.ark_last is not None:
            lx, ly, lyaw, lt = self.ark_last
            dx, dy = s.x - lx, s.y - ly
            if math.hypot(dx, dy) < 1.0:  # not the elevator teleport
                fwd = math.cos(lyaw) * dx + math.sin(lyaw) * dy
                lat = -math.sin(lyaw) * dx + math.cos(lyaw) * dy
                k = 1.0 + self.ark_scale
                dyaw = (s.yaw - lyaw + math.pi) % (2 * math.pi) - math.pi + self.ark_yaw_drift * (self.t - lt)
                a = self.ark
                a[0] += k * (math.cos(a[2]) * fwd - math.sin(a[2]) * lat)
                a[1] += k * (math.sin(a[2]) * fwd + math.cos(a[2]) * lat)
                a[2] += dyaw
        self.ark_last = (s.x, s.y, s.yaw, self.t)
        ax, ay, ayaw = self.ark
        cx, cy = ax + c.x * math.cos(ayaw), ay + c.x * math.sin(ayaw)
        pose = arkit_camera_pose(cx, cy, c.z, ayaw, c.pitch)
        self.r3d.send(rgb, d, conf, (c.fx, c.fy, c.cx, c.cy), pose)

    def publish_cameras(self):
        if self.iphone:
            self.publish_iphone()
            return
        s = self.chassis.s
        sides = ("left",) if self.mono else ("left", "right")
        for side in sides:
            img = self.renderer.render(self.world.here, s.x, s.y, s.yaw, side, t=self.t)
            stamp = _stamp(self.t)
            m = Image()
            m.header.stamp, m.header.frame_id = stamp, f"{side}_camera_optical_frame"
            m.height, m.width, m.encoding, m.step = img.shape[0], img.shape[1], "mono8", img.shape[1]
            m.data = img.tobytes()
            c = self.cam
            info = CameraInfo()
            info.header = m.header
            info.width, info.height, info.distortion_model = c.width, c.height, "plumb_bob"
            info.d = [0.0] * 5
            info.k = [c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            tx = 0.0 if side == "left" else -c.fx * c.baseline
            info.p = [c.fx, 0.0, c.cx, tx, 0.0, c.fy, c.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.img_pub[side].publish(m)
            self.info_pub[side].publish(info)

    def publish_perception(self, k: int):
        s = self.chassis.s
        here = self.world.here
        cam = self.camera_xy()
        if self.emulate_people:
            self.person_pub.publish(Float32(data=nearest_person(here, s.x, s.y, s.yaw, self.t, cam)))
        if self.emulate_signs and k % 5 == 0:
            for text, bearing in self.signs.reads(here, s.yaw, self.t, cam):
                self.sign_pub.publish(String(data=json.dumps({"text": text, "conf": 0.9, "stamp": self.t, "bearing": bearing})))
        if self.emulate_floor and k % 2 == 0:
            obs, free = floor_clouds(here, s.x, s.y, s.yaw, self.t, cam)
            header = Header(stamp=_stamp(self.t), frame_id="base_link")
            self.floor_obs_pub.publish(point_cloud2.create_cloud_xyz32(header, obs))
            self.floor_free_pub.publish(point_cloud2.create_cloud_xyz32(header, free))

    def publish_truth(self):
        s = self.chassis.s
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = _stamp(self.t), "map"
        p.pose.position.x, p.pose.position.y = s.x, s.y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(s.yaw / 2), math.cos(s.yaw / 2)
        self.truth_pub.publish(p)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = p.header.stamp, "sim_truth", "base_link_truth"
        tf.transform.translation.x, tf.transform.translation.y = s.x, s.y
        tf.transform.rotation = p.pose.orientation
        self.tf.sendTransform(tf)

    def publish_status(self):
        s = self.chassis.s
        self.status_pub.publish(String(data=json.dumps({
            "t": round(self.t, 2), "floor": self.world.floor, "x": round(s.x, 2), "y": round(s.y, 2),
            "yaw": round(s.yaw, 3), "v": round(s.v, 3), "w": round(s.w, 3), "collisions": s.collisions,
            "collision_at": s.history[-5:], "bumper_m": None if not math.isfinite(self.echo_m) else round(self.echo_m, 2),
            "controller_pwm": [self.fw.pwm(self.chassis.wiring.steer), self.fw.pwm(self.chassis.wiring.throttle)],
            "steer": round(s.steer, 3),
            "phone_posts": self.phone.posted, "phone_failures": self.phone.failed,
            "rig_done": None if self.rig is None else self.rig["done"],
            "iphone_frames": self.r3d.sent if self.iphone else None,
        })))

    def run(self):
        wall0 = time.monotonic()
        next_clock = 0.0
        next_cam = 0.5  # let the drivers start before the first frame
        next_10hz = 0.0
        k = 0
        while rclpy.ok():
            self.physics_step()
            if self.t >= next_clock:
                next_clock += CLOCK_DT
                self.clock_pub.publish(Clock(clock=_stamp(self.t)))
                rclpy.spin_once(self, timeout_sec=0.0)
            if self.t >= next_10hz:
                next_10hz += 0.1
                k += 1
                self.echo_m = self.ultrasonic()
                self.gps_port.write(nmea_sentences(utc_at(self.t), None, None))  # indoors: no GPS fix
                self.publish_perception(k)
                self.publish_truth()
                if k % 5 == 0:
                    self.publish_status()
            if self.t >= next_cam:
                next_cam += 1.0 / self.camera_rate
                self.publish_cameras()
            # Never faster than max_rtf times real time.
            ahead = self.t / self.rtf - (time.monotonic() - wall0)
            if ahead > 0.002:
                time.sleep(ahead)

    def close(self):
        self.motor_port.close()
        self.gps_port.close()
        self.fw.close()
        if self.iphone:
            self.r3d.close()


def main(args=None):
    rclpy.init(args=args)
    node = HilSim()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
