"""Stand-ins for the perception nodes that need learned models.

YOLO (people), PaddleOCR (room signs) and SegFormer (floor) need model files
this simulation may not have. These functions give what a good detector would
report, from the simulated world: only what the left camera can see (field of
view, range, walls in the way), with misses. The messages are the real nodes'
messages, so everything downstream (gate, mission, costmaps) is the real code.
Vanishing-point heading and stereo depth are not emulated: their real nodes
run on the rendered images.
"""

from __future__ import annotations

import math
import random

import numpy as np

from .world import FloorWorld

HALF_FOV = math.radians(35.0)  # the 640 px wide 1080p webcam at fx 453


def _visible(world: FloorWorld, cx: float, cy: float, yaw: float, tx: float, ty: float,
             max_range: float, t: float, slack: float = 0.3, half_fov: float = HALF_FOV) -> float | None:
    """Distance to (tx, ty) if the camera at (cx, cy, yaw) can see it, else None."""
    d = math.hypot(tx - cx, ty - cy)
    if d > max_range:
        return None
    bearing = math.atan2(ty - cy, tx - cx)
    if abs((bearing - yaw + math.pi) % (2 * math.pi) - math.pi) > half_fov:
        return None
    hit = world.raycast(cx, cy, np.array([bearing]), max_range=d + 1.0, t=t, people=False)
    return d if hit.distance[0] >= d - slack else None


def nearest_person(world: FloorWorld, x: float, y: float, yaw: float, t: float, cam: tuple[float, float],
                   max_range: float = 15.0) -> float:
    """``people/nearest``: distance (m) from the car to the nearest visible person, inf if none."""
    best = math.inf
    for px, py, r in world.people_at(t):
        if _visible(world, cam[0], cam[1], yaw, px, py, max_range, t) is not None:
            best = min(best, max(0.0, math.hypot(px - x, py - y) - r))
    return best


class SignReader:
    """``signs``: room numbers on the walls the camera passes, read with some misses."""

    def __init__(self, doors: dict[str, list[tuple[str, float, float]]], read_prob: float = 0.6,
                 max_range: float = 4.0, seed: int = 2):
        self.doors = doors  # floor -> [(text, x, y)]
        self.read_prob = read_prob
        self.max_range = max_range
        self.rand = random.Random(seed)

    def reads(self, world: FloorWorld, yaw: float, t: float, cam: tuple[float, float]) -> list[tuple[str, float]]:
        """(text, bearing from the camera axis, left positive) for each sign read now."""
        out = []
        for text, dx, dy in self.doors.get(world.floor, []):
            if abs(dx - cam[0]) > self.max_range or abs(dy - cam[1]) > self.max_range:
                continue
            if _visible(world, cam[0], cam[1], yaw, dx, dy, self.max_range, t, half_fov=math.radians(30)) is None:
                continue
            if self.rand.random() < self.read_prob:
                bearing = (math.atan2(dy - cam[1], dx - cam[0]) - yaw + math.pi) % (2 * math.pi) - math.pi
                out.append((text, bearing))
        return out


def floor_clouds(world: FloorWorld, x: float, y: float, yaw: float, t: float, cam: tuple[float, float],
                 max_range: float = 5.0, rays: int = 48, obstacle_height: float = 0.1):
    """``floor/obstacles`` and ``floor/free`` (base_link points) as floor segmentation would give."""
    angles = yaw + np.linspace(-HALF_FOV, HALF_FOV, rays)
    hit = world.raycast(cam[0], cam[1], angles, max_range=max_range, t=t)
    c, s = math.cos(-yaw), math.sin(-yaw)
    obstacles, free = [], []
    for a, d in zip(angles, hit.distance):
        reach = min(d, max_range)
        for dist, out, z in ((reach - 0.05, free, 0.0), (d, obstacles, obstacle_height)):
            if out is obstacles and not math.isfinite(d):
                continue
            wx, wy = cam[0] + dist * math.cos(a) - x, cam[1] + dist * math.sin(a) - y
            out.append((c * wx - s * wy, s * wx + c * wy, z))
    return obstacles, free
