"""Pixels to points on the ground plane, for camera-only obstacle marking."""

from __future__ import annotations

import math

import numpy as np


def rays(K: np.ndarray, u, v) -> np.ndarray:
    """Unit-depth rays (optical frame: x right, y down, z forward) for pixel coordinates."""
    u = np.asarray(u, dtype=float)
    v = np.asarray(v, dtype=float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)


def nominal_K(width: int, height: int, hfov_deg: float = 70.0):
    """Pinhole intrinsics from the image size and horizontal field of view (no calibration yet)."""
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.0]])


def mount_pose(x: float, y: float, z: float, pitch: float, yaw: float):
    """(R, t) of a camera's optical frame in base_link from its URDF mount (pitch < 0: tilted up)."""
    R0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)  # optical (x right, y down, z ahead) -> link
    cp, sp, cy, sy = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ R0, np.array([x, y, z], dtype=float)


def project_to_ground(K, R, t, u, v, max_range: float = 8.0):
    """Intersect pixel rays with z=0 in the base frame.

    ``R``, ``t`` place the camera's optical frame in base_link (point_base =
    R @ point_optical + t). Returns (N, 3) points and a validity mask; rays that
    point at or above the horizon, or land beyond ``max_range``, are invalid.
    """
    d = rays(K, u, v) @ np.asarray(R, dtype=float).T
    t = np.asarray(t, dtype=float)
    dz = d[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):  # rays at the horizon: masked below
        s = -t[2] / dz
        pts = t + s[..., None] * d
    ok = (dz < -1e-6) & (s > 0) & (np.hypot(pts[..., 0], pts[..., 1]) <= max_range)
    return pts, ok


def free_boundary(drivable: np.ndarray, start_row: int = 0, step: int = 4):
    """For every ``step``-th column, the lowest row where drivable floor ends.

    Scans up from the bottom of the image. Returns (columns, rows, hit) where
    ``hit`` is False when the whole column above ``start_row`` is drivable.
    """
    h, w = drivable.shape
    cols = np.arange(step // 2, w, step)
    rows = np.full(cols.shape, start_row, dtype=int)
    hit = np.zeros(cols.shape, dtype=bool)
    sub = drivable[start_row:, cols]  # (H', C)
    blocked = ~sub[::-1]  # bottom row first
    any_block = blocked.any(axis=0)
    first = blocked.argmax(axis=0)
    rows[any_block] = h - 1 - first[any_block]
    hit[any_block] = True
    return cols, rows, hit


def optical_to_base_rotation(pitch_down: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    """Rotation from the optical frame to base_link for a forward camera tilted down by ``pitch_down``."""
    # Optical (x right, y down, z fwd) to body (x fwd, y left, z up).
    base_from_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    cp, sp = np.cos(pitch_down), np.sin(pitch_down)
    pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])  # positive = nose down about +y
    cy, sy = np.cos(yaw), np.sin(yaw)
    yawm = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return yawm @ pitch @ base_from_optical


def quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
