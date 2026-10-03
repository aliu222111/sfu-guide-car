"""Mission node: owns the trip (spec section 8).

Topics in:  guide/destination (String: "AQ 3150" or a node id), guide/confirm,
            guide/pause (Bool), guide/cancel, guide/heartbeat, guide/phone_rssi
            (Float32 dBm), guide/localized (Bool), signs (String, JSON).
Topics out: guide/state (String, JSON), guide/speech, guide/route (String, JSON),
            guide/drive_enable, guide/recovery_rotate, guide/cmd_vel_manual,
            guide/localization_mode (String, JSON), guide/set_pose,
            guide/sign_fix, guide/corridor_axis (Float64, radians).
Action:     navigate_through_poses (Nav2).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Float32, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener

from sfu_map.route import RouteGraph

from .follow import FollowMonitor, FollowParams
from .instructions import corridor_axis, waypoints
from .signs import match_sign, viewpoint
from .trip import (
    CancelNav,
    Creep,
    DriveEnable,
    Navigate,
    Recovery,
    SetLocalization,
    SetPose,
    Speak,
    StateChanged,
    Trip,
    TripParams,
)


# Latched topics: a node that starts late still gets the last value.
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def quat_to_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission")
        graph_path = self.declare_parameter("graph", "maps/route_graph.json").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        start_floor = self.declare_parameter("start_floor", "").value or None
        tp = TripParams()
        for name in vars(tp):
            setattr(tp, name, self.declare_parameter(name, getattr(tp, name)).value)
        fp = FollowParams()
        for name in vars(fp):
            setattr(fp, name, self.declare_parameter(name, getattr(fp, name)).value)

        self.rg = RouteGraph.load(Path(graph_path))
        self.trip = Trip(self.rg, tp)
        self.trip.floor = start_floor
        self.follow = FollowMonitor(fp)
        self.follow.voice_mode = tp.voice_mode
        self.get_logger().info(f"loaded {len(self.rg.nodes)} nodes, {len(self.rg.destinations)} destinations")

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.goal_seq = 0
        self.goal_handle = None
        self.creep_left = 0.0  # seconds of creep still to drive
        self.creep_speed = 0.0
        self.turn_left = 0.0  # seconds of in-place turn before the creep
        self.turn_rate = 0.0
        self.creep_turn_speed = self.declare_parameter("creep_turn_rate", 0.4).value  # rad/s
        self.enable = False
        self.recovery = False
        self.pose = None  # (x, y, yaw) in map

        pub = lambda t, typ, q=10: self.create_publisher(typ, t, q)
        self.state_pub = pub("guide/state", String)
        self.speech_pub = pub("guide/speech", String)
        self.route_pub = pub("guide/route", String, LATCHED)
        self.enable_pub = pub("guide/drive_enable", Bool)
        self.recovery_pub = pub("guide/recovery_rotate", Bool)
        self.manual_pub = pub("guide/cmd_vel_manual", Twist)
        self.locmode_pub = pub("guide/localization_mode", String, LATCHED)
        self.set_pose_pub = pub("guide/set_pose", PoseWithCovarianceStamped)
        self.sign_fix_pub = pub("guide/sign_fix", PoseWithCovarianceStamped)
        self.axis_pub = pub("guide/corridor_axis", Float64)

        sub = self.create_subscription
        sub(String, "guide/destination", self.on_destination, 10)
        sub(Empty, "guide/confirm", lambda _: self.run(self.trip.on_confirm()), 10)
        sub(Bool, "guide/pause", lambda m: self.run(self.trip.on_pause(m.data)), 10)
        sub(Empty, "guide/cancel", lambda _: self.run(self.trip.abort("cancelled")), 10)
        sub(Empty, "guide/heartbeat", lambda _: self.follow.heartbeat(self.now()), 10)
        sub(Float32, "guide/phone_rssi", lambda m: self.follow.rssi(m.data, self.now()), 10)
        sub(Float32, "guide/student_distance", lambda m: self.follow.student(m.data, self.now()), 10)
        sub(Bool, "guide/localized", self.on_localized, 10)
        sub(String, "signs", self.on_sign, 10)

        self.create_timer(0.2, self.tick)
        self.create_timer(0.1, self.creep_tick)
        self.create_timer(1.0, self.publish_state)
        # The velocity gate drops enable and recovery unless they keep arriving.
        self.create_timer(0.5, self.publish_drive_flags)
        self.create_timer(5.0, self.publish_route)
        self.last_state = {"state": "idle", "detail": ""}
        self.replan_on_fix = False
        self.sign_read_distance = float(self.declare_parameter("sign_read_distance", 2.5).value)
        self.reset_yaw_sigma = float(self.declare_parameter("reset_yaw_sigma", 0.6).value)  # rad, the compass hint's
        self.last_fix_xy = None
        self.resume_after_reset = None  # (time, (x, y)) while waiting for the EKF reset to show in TF
        self.run([SetLocalization(start_floor), DriveEnable(False)])

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- Inputs ------------------------------------------------------------------
    def on_destination(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        start = self.start_node()
        if start is None:
            self.say("I don't know where I am yet. Give me a moment.")
            return
        # Asked before the car knows where it is: this plan starts from a guess,
        # so plan again from the real position once localization arrives.
        self.replan_on_fix = not self.trip.localized
        self.run(self.trip.start(text, start))
        self.publish_route()

    def on_localized(self, msg: Bool):
        if msg.data and self.replan_on_fix and self.trip.active and not self.trip.localized:
            self.replan_on_fix = False
            # From the fix itself: the EKF reset may not have reached TF yet.
            start = self.start_node(self.last_fix_xy)
            if start is not None:
                self.get_logger().info(f"position found: planning the trip again from {start}")
                self.run(self.trip.replan_from(start))
                self.publish_route()
                # Drive once the reset has reached TF, or Nav2 plans from the old guess.
                self.resume_after_reset = (self.now(), self.last_fix_xy)
                return
        if not msg.data:
            self.resume_after_reset = None
        if self.resume_after_reset is None:
            self.run(self.trip.on_localization(msg.data))

    def publish_route(self):
        if self.trip.path and self.trip.active:
            self.route_pub.publish(
                String(data=json.dumps([{"id": n, **{k: self.rg.nodes[n][k] for k in ("x", "y", "floor", "type")}} for n in self.trip.path]))
            )

    def start_node(self, xy=None):
        if xy is None and self.pose is None:
            return None
        x, y = xy if xy is not None else self.pose[:2]
        floor = self.trip.floor
        try:
            return self.rg.nearest_node(floor, x, y, types={"corridor", "corridor_junction", "entrance", "outdoor_junction", "elevator"})
        except Exception:
            return None

    def on_sign(self, msg: String):
        try:
            read = json.loads(msg.data)
        except ValueError:
            return
        x, y = (self.pose[0], self.pose[1]) if self.pose else (None, None)
        # Not localized (cold start indoors, odometry lost) or lost after an elevator:
        # the estimate cannot be trusted, so any matching door on the floor counts.
        untrusted = self.trip.state == "lost" or not self.trip.localized
        fix = match_sign(self.rg, str(read.get("text", "")), self.trip.floor, x, y, lost=untrusted)
        if fix is None:
            return
        yaw = self.pose[2] if self.pose else 0.0
        bearing = read.get("bearing", 0.0)
        bearing = float(bearing) if isinstance(bearing, (int, float)) and math.isfinite(bearing) else 0.0
        fx, fy = viewpoint(fix.x, fix.y, yaw, bearing, self.sign_read_distance)
        self.get_logger().info(f"sign fix {fix.room} at {fix.distance:.1f} m from estimate")
        msg = self.pose_msg(fx, fy, yaw, fix.sigma, yaw_sigma=math.pi)  # a sign says nothing about heading
        self.last_fix_xy = (fx, fy)
        if untrusted:
            # The EKF rejects a measurement far from its estimate (as it should for a
            # misread sign while tracking), so an untrusted estimate is reset instead.
            # Keep the heading the EKF has (gyro, compass hint) with a realistic sigma:
            # resetting it to "unknown" made later heading updates drag the position
            # around by metres through the heading-position correlation.
            self.set_pose_pub.publish(self.pose_msg(fx, fy, yaw, fix.sigma, yaw_sigma=self.reset_yaw_sigma))
        self.sign_fix_pub.publish(msg)

    def tick(self):
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.0))
            tr = t.transform
            self.pose = (tr.translation.x, tr.translation.y, quat_to_yaw(tr.rotation))
        except TransformException:
            pass
        if self.resume_after_reset is not None:
            since, (fx, fy) = self.resume_after_reset
            arrived = self.pose is not None and math.hypot(self.pose[0] - fx, self.pose[1] - fy) < 3.0
            if arrived or self.now() - since > 3.0:
                self.resume_after_reset = None
                self.run(self.trip.on_localization(True))
        out = self.trip.on_follow(self.follow.following(self.now()))
        if self.pose is not None:
            out += self.trip.on_position(self.pose[0], self.pose[1])
            self.publish_axis()
        self.run(out)

    def publish_axis(self):
        leg = self.trip.leg
        if self.trip.state != "driving" or leg is None or leg.floor is None or len(leg.nodes) < 2:
            return
        axis = corridor_axis(self.rg, leg.nodes, self.trip.progress, self.pose[2] if self.pose else None)
        if axis is not None:
            self.axis_pub.publish(Float64(data=axis))

    # --- Actions -----------------------------------------------------------------
    def run(self, actions):
        for a in actions:
            if isinstance(a, Speak):
                self.say(a.text)
            elif isinstance(a, StateChanged):
                self.last_state = {"state": a.state, "detail": a.detail}
                self.publish_state()
                self.get_logger().info(f"state {a.state} {a.detail}")
            elif isinstance(a, Navigate):
                self.navigate(a)
            elif isinstance(a, CancelNav):
                self.cancel_nav()
                self.stop_creep()
            elif isinstance(a, SetLocalization):
                self.locmode_pub.publish(String(data=json.dumps({"floor": a.floor})))
            elif isinstance(a, SetPose):
                n = self.rg.nodes[a.node]
                yaw = self.pose[2] if self.pose else 0.0
                self.set_pose_pub.publish(self.pose_msg(n["x"], n["y"], yaw, a.sigma, yaw_sigma=math.pi))
            elif isinstance(a, Creep):
                self.creep_speed = a.speed
                self.creep_left = a.distance / max(a.speed, 0.05)
                self.turn_left = abs(a.turn) / self.creep_turn_speed
                self.turn_rate = math.copysign(self.creep_turn_speed, a.turn) if a.turn else 0.0
            elif isinstance(a, Recovery):
                self.recovery = a.on
                self.publish_drive_flags()
            elif isinstance(a, DriveEnable):
                self.enable = a.on
                if not a.on:
                    self.stop_creep()
                self.publish_drive_flags()

    def publish_drive_flags(self):
        self.enable_pub.publish(Bool(data=self.enable))
        self.recovery_pub.publish(Bool(data=self.recovery))

    def say(self, text: str):
        self.get_logger().info(f"say: {text}")
        self.speech_pub.publish(String(data=text))

    def publish_state(self):
        leg = self.trip.leg
        doc = {
            **self.last_state,
            "destination": self.trip.destination,
            "floor": self.trip.floor,
            "leg": self.trip.leg_index,
            "legs": len(self.trip.legs),
            "leg_kind": leg.kind if leg else None,
            "paused": self.trip.paused,
            "pose": list(self.pose) if self.pose else None,
        }
        self.state_pub.publish(String(data=json.dumps(doc)))

    def pose_msg(self, x, y, yaw, sigma, yaw_sigma=0.3):
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        q = yaw_to_quat(yaw)
        m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w = q
        cov = [0.0] * 36
        cov[0] = cov[7] = sigma**2
        cov[14] = cov[21] = cov[28] = 1e-3
        cov[35] = yaw_sigma**2
        m.pose.covariance = cov
        return m

    def navigate(self, a: Navigate):
        self.cancel_nav()
        poses = []
        nodes = [self.rg.nodes[n] for n in waypoints(self.rg, a.nodes)]
        # Skip the first node: it is where the car already is.
        for i in range(1, len(nodes)):
            n = nodes[i]
            ref = nodes[i - 1]
            yaw = math.atan2(n["y"] - ref["y"], n["x"] - ref["x"])
            p = PoseStamped()
            p.header.frame_id = self.map_frame
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = float(n["x"]), float(n["y"])
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = yaw_to_quat(yaw)
            poses.append(p)
        if not poses:
            self.run(self.trip.on_nav_result(True))
            return
        if not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_through_poses action server not available")
            self.run(self.trip.on_nav_result(False))
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        self.goal_seq += 1
        seq = self.goal_seq
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_goal_accepted(f, seq))

    def on_goal_accepted(self, fut, seq):
        handle = fut.result()
        if seq != self.goal_seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.run(self.trip.on_nav_result(False))
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, fut, seq):
        if seq != self.goal_seq:
            return  # superseded or cancelled
        self.goal_handle = None
        status = fut.result().status
        self.run(self.trip.on_nav_result(status == GoalStatus.STATUS_SUCCEEDED))

    def cancel_nav(self):
        self.goal_seq += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def stop_creep(self):
        if self.creep_left > 0 or self.turn_left > 0:
            self.creep_left = self.turn_left = 0.0
            self.manual_pub.publish(Twist())

    def creep_tick(self):
        """Drive the elevator creep; it pauses while the trip is paused or the student is away."""
        if self.creep_left <= 0:
            return
        if self.trip.paused or not self.trip.following:
            self.manual_pub.publish(Twist())
            return
        t = Twist()
        if self.turn_left > 0:
            self.turn_left -= 0.1
            t.angular.z = self.turn_rate
            self.manual_pub.publish(t)
            return
        self.creep_left -= 0.1
        if self.creep_left > 0:
            t.linear.x = self.creep_speed
            self.manual_pub.publish(t)
            return
        self.manual_pub.publish(t)
        self.run(self.trip.on_creep_done())

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
