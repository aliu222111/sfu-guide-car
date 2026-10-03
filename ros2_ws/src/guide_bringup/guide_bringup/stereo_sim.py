"""Synthetic stereo camera in a textured ring corridor, for testing RTAB-Map without hardware.

The car drives laps of a rectangular ring corridor (a square "racetrack" of
corridor around a solid block, like the AQ concourse). Every frame is
ray-traced: walls, floor and ceiling carry a fixed random texture, so visual
odometry has features and a second lap revisits the same views (loop
closure). Publishes rectified left/right images and camera info as
``stereo_image_proc`` would (``/stereo/left/image_rect`` ...), the phone IMU's
yaw rate on ``/phone/imu``, and ground truth on TF ``sim_truth -> base_link_truth``.

    ros2 run guide_bringup stereo_sim --ros-args -p laps:=2
"""

from __future__ import annotations

import math

import numpy as np

# Corridor: the space between an outer box and an inner block, centred on 0.
OUTER = (11.0, 7.0)  # half extents, metres
INNER = (8.5, 4.5)
HEIGHT = 2.6
W, H = 640, 360
FX = FY = 380.0
CX, CY = W / 2, H / 2
BASELINE = 0.12
CAM_Z = 0.45
TEX = 4096  # texture tile size in pixels: 41 m, longer than any wall, so no view repeats
TEX_RES = 0.01  # metres per texel


def make_texture(seed: int = 3) -> np.ndarray:
    """Grey blotches at several scales: plenty of corners for ORB/GFTT."""
    rng = np.random.default_rng(seed)
    t = np.zeros((TEX, TEX), np.float32)
    for scale, weight in ((128, 0.5), (32, 0.3), (8, 0.2)):
        small = rng.random((TEX // scale, TEX // scale)).astype(np.float32)
        t += weight * np.kron(small, np.ones((scale, scale), np.float32))
    # Sharp-edged rectangles ("posters", door frames).
    for _ in range(300 * (TEX // 1024) ** 2):
        x, y = rng.integers(0, TEX - 80, 2)
        w, h = rng.integers(10, 80, 2)
        t[y:y + h, x:x + w] = rng.random()
    return (255 * (t - t.min()) / (t.max() - t.min())).astype(np.uint8)


CORNER_R = 1.0  # m: rounded corners keep the turn rate near the car's 30 deg/s cap


def _segments():
    """The corridor centre loop, anticlockwise: straights joined by quarter arcs."""
    hx, hy = (OUTER[0] + INNER[0]) / 2, (OUTER[1] + INNER[1]) / 2
    r = CORNER_R
    segs = []
    # (kind, length, start x, start y, start heading, arc centre or None)
    corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
    starts = [(hx, -hy + r, math.pi / 2), (hx - r, hy, math.pi), (-hx, hy - r, -math.pi / 2), (-hx + r, -hy, 0.0)]
    lengths = [2 * (hy - r), 2 * (hx - r), 2 * (hy - r), 2 * (hx - r)]
    for (x0, y0, h0), L, (cx, cy) in zip(starts, lengths, corners):
        segs.append(("line", L, x0, y0, h0, None))
        ex, ey = x0 + L * math.cos(h0), y0 + L * math.sin(h0)
        centre = (ex + r * math.cos(h0 + math.pi / 2), ey + r * math.sin(h0 + math.pi / 2))
        segs.append(("arc", r * math.pi / 2, ex, ey, h0, centre))
    return segs


def perimeter() -> float:
    return sum(sg[1] for sg in _segments())


def on_corner(s: float) -> bool:
    s = s % perimeter()
    for kind, L, *_ in _segments():
        if s <= L:
            return kind == "arc"
        s -= L
    return False


def centreline(s: float) -> tuple[float, float, float]:
    """Position and heading at arc length ``s`` along the loop."""
    s = s % perimeter()
    for kind, L, x0, y0, h0, centre in _segments():
        if s <= L:
            if kind == "line":
                return x0 + s * math.cos(h0), y0 + s * math.sin(h0), h0
            a = s / CORNER_R  # turning left
            cx, cy = centre
            ang0 = h0 - math.pi / 2
            return cx + CORNER_R * math.cos(ang0 + a), cy + CORNER_R * math.sin(ang0 + a), h0 + a
        s -= L
    raise AssertionError("unreachable")


class Renderer:
    def __init__(self, seed: int = 3):
        self.tex = make_texture(seed)
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        # Camera rays in the camera frame: x forward, y left, z up (level camera).
        self.rays = np.stack([np.ones_like(u, float), -(u - CX) / FX, -(v - CY) / FY], -1)

    def _texture(self, a, b, face):
        # A wall spans 2.6 m of b; 497 texels (5 m) per face gives every wall its own
        # band of the texture, so no two walls look alike.
        i = ((a / TEX_RES).astype(np.int64) + 1009 * face) % TEX
        j = ((b / TEX_RES).astype(np.int64) + 497 * face) % TEX
        return self.tex[j, i]

    def render(self, x: float, y: float, yaw: float) -> np.ndarray:
        c, s = math.cos(yaw), math.sin(yaw)
        d = self.rays
        dx = c * d[..., 0] - s * d[..., 1]
        dy = s * d[..., 0] + c * d[..., 1]
        dz = d[..., 2]
        best = np.full(dx.shape, np.inf)
        img = np.zeros(dx.shape, np.uint8)
        ox, oy, oz = x, y, CAM_Z

        def consider(t, valid, shade):
            nonlocal best
            ok = valid & (t > 1e-3) & (t < best)
            best = np.where(ok, t, best)
            img[ok] = shade[ok]

        with np.errstate(divide="ignore", invalid="ignore"):
            # Floor and ceiling.
            for zp, face in ((0.0, 0), (HEIGHT, 1)):
                t = (zp - oz) / dz
                px, py = ox + t * dx, oy + t * dy
                consider(t, np.isfinite(t), self._texture(px + 50, py + 50, face))
            # Outer walls (seen from inside) and inner block walls (seen from outside).
            for X, Y, inner, base in ((OUTER[0], OUTER[1], False, 2), (INNER[0], INNER[1], True, 6)):
                for sign, axis, k in ((1, 0, 0), (-1, 0, 1), (1, 1, 2), (-1, 1, 3)):
                    lim = X if axis == 0 else Y
                    o = ox if axis == 0 else oy
                    dd = dx if axis == 0 else dy
                    t = (sign * lim - o) / dd
                    px, py, pz = ox + t * dx, oy + t * dy, oz + t * dz
                    along = py if axis == 0 else px
                    other = Y if axis == 0 else X
                    valid = np.isfinite(t) & (np.abs(along) <= other) & (pz >= 0) & (pz <= HEIGHT)
                    consider(t, valid, self._texture(along + 50, pz + 50, base + k))
        return img

    def stereo(self, x, y, yaw):
        # Left camera at +baseline/2 to the left (base_link y), right at -baseline/2.
        lx, ly = x - math.sin(yaw) * BASELINE / 2, y + math.cos(yaw) * BASELINE / 2
        rx, ry = x + math.sin(yaw) * BASELINE / 2, y - math.cos(yaw) * BASELINE / 2
        return self.render(lx, ly, yaw), self.render(rx, ry, yaw)


def main(args=None):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class StereoSim(Node):
        def __init__(self):
            super().__init__("stereo_sim")
            def num(name, default):  # accept start_s:=0 as well as start_s:=0.0
                return float(self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value)

            self.speed = num("speed", 0.4)
            # Slower round corners, as the car drives them. This container's CPU runs
            # stereo odometry at about 2 Hz; at full speed a corner turns 11 deg per
            # processed frame, far more than the car's 15-30 fps cameras see.
            self.corner_speed = num("corner_speed", 0.15)
            self.rate = num("rate_hz", 4.0)
            self.laps = num("laps", 2.0)
            self.s0 = num("start_s", 0.0)  # metres along the loop to start at
            self.r = Renderer()
            pub = self.create_publisher
            self.pubs = {k: pub(Image, f"/stereo/{k}/image_rect", 5) for k in ("left", "right")}
            self.infos = {k: pub(CameraInfo, f"/stereo/{k}/camera_info", 5) for k in ("left", "right")}
            self.imu = pub(Imu, "/phone/imu", 20) if self.declare_parameter("imu", True).value else None
            self.tf = TransformBroadcaster(self)
            self.s = self.s0
            self.last_yaw = None
            # The rendered cameras are level, CAM_Z up, BASELINE apart: say so on TF
            # (the car's URDF pitches them, these images do not).
            static = []
            for side, y in (("left", BASELINE / 2), ("right", -BASELINE / 2)):
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id, t.child_frame_id = "base_link", f"{side}_camera_optical_frame"
                t.transform.translation.y, t.transform.translation.z = y, CAM_Z
                # optical (z fwd, x right, y down) from base (x fwd, y left, z up)
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = -0.5, 0.5, -0.5, 0.5
                static.append(t)
            self.static_tf = StaticTransformBroadcaster(self)
            self.static_tf.sendTransform(static)
            self.create_timer(1.0 / self.rate, self.tick)

        def info(self, side, stamp):
            m = CameraInfo()
            m.header.stamp, m.header.frame_id = stamp, f"{side}_camera_optical_frame"
            m.width, m.height = W, H
            m.distortion_model = "plumb_bob"
            m.d = [0.0] * 5
            m.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
            m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            tx = 0.0 if side == "left" else -FX * BASELINE
            m.p = [FX, 0.0, CX, tx, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
            return m

        def publish_imu(self, stamp, yaw):
            # A level phone at rest except for yaw: gravity on +z, yaw rate from the path.
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, "base_link"
            if self.last_yaw is not None:
                dyaw = (yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi
                imu.angular_velocity.z = dyaw * self.rate
            imu.linear_acceleration.z = 9.81
            imu.orientation_covariance[0] = -1.0
            self.imu.publish(imu)
            self.last_yaw = yaw

        def tick(self):
            if self.s - self.s0 > self.laps * perimeter():
                return
            x, y, yaw = centreline(self.s)
            stamp = self.get_clock().now().to_msg()
            left, right = self.r.stereo(x, y, yaw)
            for side, img in (("left", left), ("right", right)):
                m = Image()
                m.header.stamp, m.header.frame_id = stamp, f"{side}_camera_optical_frame"
                m.height, m.width, m.encoding, m.step = H, W, "mono8", W
                m.data = img.tobytes()
                self.pubs[side].publish(m)
                self.infos[side].publish(self.info(side, stamp))
            if self.imu is not None:
                self.publish_imu(stamp, yaw)
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "sim_truth", "base_link_truth"
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.tf.sendTransform(t)
            self.s += (self.corner_speed if on_corner(self.s) else self.speed) / self.rate

    rclpy.init(args=args)
    node = StereoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
