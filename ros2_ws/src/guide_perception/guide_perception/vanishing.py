"""Corridor heading from the vanishing point of wall-floor lines (spec section 6).

The corridor's straight edges meet at one point in the image. Its horizontal
offset from the image centre gives the car's yaw relative to the corridor
axis: ``yaw_car = yaw_corridor + atan((vp_x - cx) / fx)``. The route graph
knows the corridor's absolute direction, so this is an absolute heading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class VanishingPoint:
    x: float
    y: float
    inliers: int
    lines: int


def detect_lines(gray: np.ndarray, min_length: float = 40.0) -> np.ndarray:
    """(N, 4) line segments x1, y1, x2, y2 from Canny + probabilistic Hough."""
    import cv2

    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
    segs = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40, minLineLength=int(min_length), maxLineGap=8)
    return np.zeros((0, 4)) if segs is None else segs[:, 0, :].astype(float)


def oblique(segs: np.ndarray, min_deg: float = 12.0, max_deg: float = 78.0) -> np.ndarray:
    """Keep segments that are neither near horizontal nor near vertical."""
    if len(segs) == 0:
        return segs
    ang = np.degrees(np.abs(np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])))
    ang = np.minimum(ang, 180 - ang)
    return segs[(ang >= min_deg) & (ang <= max_deg)]


def _homog(segs):
    p1 = np.column_stack([segs[:, 0], segs[:, 1], np.ones(len(segs))])
    p2 = np.column_stack([segs[:, 2], segs[:, 3], np.ones(len(segs))])
    lines = np.cross(p1, p2)
    return lines / np.linalg.norm(lines[:, :2], axis=1, keepdims=True)


def _angle_error(segs, vp):
    """Angle between each segment and the line from its midpoint to the point."""
    mx, my = (segs[:, 0] + segs[:, 2]) / 2, (segs[:, 1] + segs[:, 3]) / 2
    d1 = np.arctan2(segs[:, 3] - segs[:, 1], segs[:, 2] - segs[:, 0])
    d2 = np.arctan2(vp[1] - my, vp[0] - mx)
    e = np.abs((d1 - d2 + np.pi / 2) % np.pi - np.pi / 2)
    return e


def find_vanishing_point(segs: np.ndarray, iterations: int = 300, tol_deg: float = 2.0, min_inliers: int = 4, seed: int = 0):
    """RANSAC over segment pairs, then least squares on the inliers."""
    n = len(segs)
    if n < min_inliers:
        return None
    rng = np.random.default_rng(seed)
    L = _homog(segs)
    tol = math.radians(tol_deg)
    best, best_inl = None, None
    for _ in range(iterations):
        i, j = rng.choice(n, 2, replace=False)
        p = np.cross(L[i], L[j])
        if abs(p[2]) < 1e-9:
            continue
        vp = p[:2] / p[2]
        inl = _angle_error(segs, vp) < tol
        if best_inl is None or inl.sum() > best_inl.sum():
            best, best_inl = vp, inl
    if best is None or best_inl.sum() < min_inliers:
        return None
    # Both sides of the corridor must contribute: slopes of both signs.
    s = segs[best_inl]
    slope = (s[:, 3] - s[:, 1]) * (s[:, 2] - s[:, 0])
    if not ((slope > 0).any() and (slope < 0).any()):
        return None
    A = L[best_inl][:, :2]
    b = -L[best_inl][:, 2]
    vp, *_ = np.linalg.lstsq(A, b, rcond=None)
    return VanishingPoint(float(vp[0]), float(vp[1]), int(best_inl.sum()), n)


def relative_yaw(vp_x: float, fx: float, cx: float) -> float:
    """Car yaw minus corridor yaw (positive = car turned left of the corridor)."""
    return math.atan2(vp_x - cx, fx)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def absolute_yaw(corridor_axis: float, rel: float, current_yaw: float) -> float:
    """Pick the corridor direction (axis or axis + pi) closest to the current estimate."""
    candidates = [wrap(corridor_axis + rel), wrap(corridor_axis + math.pi + rel)]
    return min(candidates, key=lambda y: abs(wrap(y - current_yaw)))
