"""Visual odometry from one camera by watching the floor (spec section 18, one-camera fallback).

With one camera nothing measures depth, but the floor is a known plane: the
camera sits at a fixed, measured height and tilt, so every floor pixel maps to
one point on the ground (``ground.project_to_ground``). Track corners in the
part of the image that sees floor between two frames, put both ends of each
track on the ground in the car's frame, and the rigid 2D motion that carries
one set onto the other is the car's motion, in metres, no scale ambiguity.
RANSAC drops tracks on walls, people and furniture: they are not on the plane
and do not fit the floor's motion.

Needs a floor with some texture (carpet, terrazzo, tiles, scuffed concrete) in
the 0.5 to 4 m band in front of the car. Plain polished floor gives too few
corners, which reads as lost, the same as stereo odometry on a blank wall.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .ground import project_to_ground


@dataclass
class GroundOdomParams:
    min_range: float = 0.4  # m: closer floor is out of view or blurred
    max_range: float = 4.0  # m: farther, one pixel covers too much floor
    max_corners: int = 400
    corner_quality: float = 0.005
    corner_spacing: int = 7  # px
    min_tracks: int = 30
    min_inliers: int = 20
    inlier_tol: float = 0.03  # m on the ground
    ransac_iters: int = 200
    max_step: float = 0.5  # m between processed frames: more is a bad fit
    max_turn: float = 0.6  # rad between processed frames


@dataclass
class Step:
    """Motion of the car from the previous frame to this one, in the previous frame's base_link."""

    dx: float
    dy: float
    dyaw: float
    inliers: int
    tracks: int


def rigid_2d(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares rotation R and translation t with R @ a_i + t ~ b_i (Kabsch, 2D)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    H = (a - ca).T @ (b - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    return R, cb - R @ ca


def ransac_rigid(a: np.ndarray, b: np.ndarray, tol: float, iters: int, rng: np.random.Generator):
    """Robust ``rigid_2d``: returns (R, t, inlier mask) or None with fewer than 2 points."""
    n = len(a)
    if n < 2:
        return None
    best = None
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        if np.linalg.norm(a[i] - a[j]) < 0.1:  # too close together to fix the rotation
            continue
        R, t = rigid_2d(a[[i, j]], b[[i, j]])
        inl = np.linalg.norm(a @ R.T + t - b, axis=1) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 2:
        return None
    R, t = rigid_2d(a[best], b[best])
    inl = np.linalg.norm(a @ R.T + t - b, axis=1) < tol  # refit, then recount
    return R, t, inl


class GroundOdometry:
    """Feed consecutive grey frames to ``update``; each returns a ``Step`` or None (lost)."""

    def __init__(self, K, R_cam, t_cam, shape: tuple[int, int], params: GroundOdomParams | None = None, seed: int = 0):
        self.p = params or GroundOdomParams()
        self.K, self.R_cam, self.t_cam = np.asarray(K, float), np.asarray(R_cam, float), np.asarray(t_cam, float)
        h, w = shape
        v, u = np.mgrid[0:h, 0:w]
        pts, ok = project_to_ground(self.K, self.R_cam, self.t_cam, u.ravel(), v.ravel(), self.p.max_range)
        rng_ = np.hypot(pts[:, 0], pts[:, 1])
        mask = ok & (rng_ >= self.p.min_range)
        self.mask = (mask.reshape(h, w) * 255).astype(np.uint8)
        if not self.mask.any():
            raise ValueError("the camera sees no floor between min_range and max_range: check its height and tilt")
        self.prev = None
        self.rng = np.random.default_rng(seed)

    def ground(self, uv: np.ndarray):
        pts, ok = project_to_ground(self.K, self.R_cam, self.t_cam, uv[:, 0], uv[:, 1], self.p.max_range)
        return pts[:, :2], ok

    def update(self, gray: np.ndarray) -> Step | None:
        prev, self.prev = self.prev, gray
        if prev is None:
            return None
        p = self.p
        c0 = cv2.goodFeaturesToTrack(prev, p.max_corners, p.corner_quality, p.corner_spacing, mask=self.mask)
        if c0 is None or len(c0) < p.min_tracks:
            return None
        c1, st, _ = cv2.calcOpticalFlowPyrLK(prev, gray, c0, None, winSize=(21, 21), maxLevel=3)
        # Forward-backward check: keep tracks that come back to where they started.
        back, st2, _ = cv2.calcOpticalFlowPyrLK(gray, prev, c1, None, winSize=(21, 21), maxLevel=3)
        good = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((back - c0).reshape(-1, 2), axis=1) < 1.0)
        a_uv, b_uv = c0.reshape(-1, 2)[good], c1.reshape(-1, 2)[good]
        if len(a_uv) < p.min_tracks:
            return None
        a, ok_a = self.ground(a_uv)
        b, ok_b = self.ground(b_uv)
        both = ok_a & ok_b
        a, b = a[both], b[both]
        if len(a) < p.min_tracks:
            return None
        # A floor point fixed in the world: seen at a (previous car frame), then
        # at b (current frame). The car moved by T with a = T(b): fit b -> a.
        fit = ransac_rigid(b, a, p.inlier_tol, p.ransac_iters, self.rng)
        if fit is None:
            return None
        R, t, inl = fit
        dyaw = math.atan2(R[1, 0], R[0, 0])
        if inl.sum() < p.min_inliers or np.hypot(*t) > p.max_step or abs(dyaw) > p.max_turn:
            return None
        return Step(float(t[0]), float(t[1]), dyaw, int(inl.sum()), int(len(a)))


def compose(pose: tuple[float, float, float], step: Step) -> tuple[float, float, float]:
    """Pose (x, y, yaw) in the odometry frame after ``step``."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * step.dx - s * step.dy, y + s * step.dx + c * step.dy, yaw + step.dyaw
