"""iPhone (LiDAR) as the front camera: ARKit pose and Record3D frames to ROS (spec section 19).

Record3D streams, per frame: an RGB image, a LiDAR depth image in metres
(smaller than the RGB), a confidence image (0 low, 1 medium, 2 high), the RGB
camera's intrinsics and the camera pose from ARKit.

ARKit's world: y up (gravity), x and z level, the origin and heading where the
session started. Its camera: x right, y up, z backwards (the camera looks
along -z). The ROS frames here: x forward, y left, z up.

The phone is mounted upright, facing forward, at (mount_x, mount_y) on
base_link. Only the planar pose matters: the car stays on the floor.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


def quat_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def base_pose(qx, qy, qz, qw, tx, ty, tz, mount_x: float = 0.0, mount_y: float = 0.0) -> tuple[float, float, float]:
    """Planar (x, y, yaw) of base_link in the odom frame ARKit's world defines.

    ARKit world -> ROS: x_ros = -z_ark, y_ros = -x_ark. The heading is that of
    the camera's viewing direction (-z column), so it does not depend on whether
    the phone is upright or on its side.
    """
    look = -quat_matrix(qx, qy, qz, qw)[:, 2]  # viewing direction in ARKit world
    yaw = math.atan2(-look[0], -look[2])
    cx, cy = -tz, -tx
    c, s = math.cos(yaw), math.sin(yaw)
    return cx - c * mount_x + s * mount_y, cy - s * mount_x - c * mount_y, yaw


def matrix_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """(qx, qy, qz, qw) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def arkit_camera_pose(x: float, y: float, z: float, yaw: float, pitch: float) -> tuple[float, ...]:
    """ARKit (qx, qy, qz, qw, tx, ty, tz) of a camera at ROS (x, y, z) looking along
    ``yaw``, tilted ``pitch`` nose down. The inverse of ``base_pose`` (the simulator's phone)."""
    cp, sp, c, s = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    look = np.array([c * cp, s * cp, -sp])  # ROS
    right = np.array([s, -c, 0.0])
    up = np.cross(right, look)
    ros_to_ark = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]])  # x_ark = -y, y_ark = z, z_ark = -x
    R = ros_to_ark @ np.stack([right, up, -look], axis=1)
    return (*matrix_quat(R), -y, z, -x)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Twist:
    vx: float
    vy: float
    wz: float


class TwistTracker:
    """Body-frame velocity from successive poses, over a short window.

    Differencing poses 1/30 s apart turns millimetre jitter into centimetres
    per second, so each velocity spans at least ``window`` seconds. ARKit can
    relocalise and jump; a step faster than the car can move resets the
    tracker instead of reporting a spike.
    """

    def __init__(self, window: float = 0.1, max_speed: float = 2.0, max_rate: float = 3.0):
        self.window = window
        self.max_speed = max_speed
        self.max_rate = max_rate
        self.hist: deque = deque(maxlen=200)
        self.jumps = 0

    def update(self, t: float, x: float, y: float, yaw: float) -> Twist | None:
        if self.hist:
            t1, x1, y1, yaw1 = self.hist[-1]
            dt = t - t1
            if dt <= 0:
                return None
            if math.hypot(x - x1, y - y1) / dt > self.max_speed or abs(wrap(yaw - yaw1)) / dt > self.max_rate:
                self.hist.clear()
                self.jumps += 1
        self.hist.append((t, x, y, yaw))
        old = None
        for h in reversed(self.hist):
            if t - h[0] >= self.window:
                old = h
                break
        if old is None:
            return None
        t0, x0, y0, yaw0 = old
        dt = t - t0
        dyaw = wrap(yaw - yaw0)
        mid = yaw0 + dyaw / 2
        dx, dy = x - x0, y - y0
        c, s = math.cos(mid), math.sin(mid)
        return Twist((c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt)


def rotate_image(img: np.ndarray, quarter_turns: int) -> np.ndarray:
    """Rotate counter-clockwise by 90 degrees ``quarter_turns`` times."""
    return np.ascontiguousarray(np.rot90(img, quarter_turns % 4))


def rotate_K(K: np.ndarray, width: int, height: int, quarter_turns: int) -> np.ndarray:
    """Intrinsics of an image of (width, height) after ``rotate_image``."""
    K = K.copy()
    for _ in range(quarter_turns % 4):
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        # Counter-clockwise: new u = v, new v = (width - 1) - u.
        K = np.array([[fy, 0, cy], [0, fx, width - 1 - cx], [0, 0, 1.0]])
        width, height = height, width
    return K


@dataclass
class Rgbd:
    bgr: np.ndarray  # out_width wide
    depth: np.ndarray  # float32 metres, registered to bgr, 0 where unknown
    K: np.ndarray  # for bgr and depth
    depth_small: np.ndarray  # the LiDAR's own resolution, 0 where unknown
    K_small: np.ndarray


def prepare(rgb: np.ndarray, depth: np.ndarray, conf: np.ndarray | None, K: np.ndarray, out_width: int = 640,
            min_confidence: int = 2, max_depth: float = 5.0, quarter_turns: int = 0) -> Rgbd:
    """Resize the RGB, register the depth to it, drop unconfident or far depth.

    Record3D's intrinsics are for the RGB image. The LiDAR depth covers the same
    view at a lower resolution, so its intrinsics are the RGB ones scaled.
    """
    h, w = rgb.shape[:2]
    if quarter_turns % 4:
        K = rotate_K(K, w, h, quarter_turns)
        rgb, depth = rotate_image(rgb, quarter_turns), rotate_image(depth, quarter_turns)
        conf = None if conf is None or conf.size == 0 else rotate_image(conf, quarter_turns)
        h, w = rgb.shape[:2]
    depth = depth.astype(np.float32, copy=True)
    bad = ~np.isfinite(depth) | (depth <= 0) | (depth > max_depth)
    if conf is not None and conf.shape == depth.shape:
        bad |= conf < min_confidence
    depth[bad] = 0.0
    scale = out_width / w
    out_h = int(round(h * scale))
    K_out = K.copy()
    K_out[:2] *= scale
    bgr = cv2.cvtColor(cv2.resize(rgb, (out_width, out_h), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2BGR)
    reg = cv2.resize(depth, (out_width, out_h), interpolation=cv2.INTER_NEAREST)
    dh, dw = depth.shape
    K_small = K.copy()
    K_small[0] *= dw / w
    K_small[1] *= dh / h
    K_small[2] = (0, 0, 1)
    return Rgbd(bgr, reg, K_out, depth, K_small)


def depth_points(depth: np.ndarray, K: np.ndarray, stride: int = 2) -> np.ndarray:
    """(N, 3) points in the camera's optical frame (x right, y down, z forward)."""
    d = depth[::stride, ::stride]
    v, u = np.nonzero(d > 0)
    z = d[v, u]
    u = u * stride
    v = v * stride
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=1).astype(np.float32)
