"""The simulated building: walls from the floor grids, plus things the map does not have.

Walls come from the same 5 cm Nav2 grids the car plans on (``maps/floors``),
so the simulated corridors are the GIS corridors. On top of them the world
adds what a real corridor has and the map does not: clutter (benches, bins,
sign boards: full-height boxes that the stereo camera must see and Nav2 must
avoid) and people walking back and forth. Ray casting serves the cameras,
the ultrasonic sensor and the perception stand-ins.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass
class Grid:
    occ: np.ndarray  # bool, True = wall; row 0 is the top (largest y)
    res: float
    ox: float
    oy: float

    @classmethod
    def load(cls, yaml_path: Path) -> "Grid":
        meta = yaml.safe_load(Path(yaml_path).read_text())
        img = cv2.imread(str(Path(yaml_path).parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(yaml_path)
        p = img.astype(np.float32) / 255.0
        if not meta.get("negate", 0):
            p = 1.0 - p  # dark = occupied
        occ = p > meta.get("occupied_thresh", 0.65)
        unknown = (p > meta.get("free_thresh", 0.25)) & ~occ
        return cls(occ | unknown, float(meta["resolution"]), float(meta["origin"][0]), float(meta["origin"][1]))

    def cell(self, x, y):
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        row = self.occ.shape[0] - 1 - np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        return row, col

    def occupied(self, x, y):
        row, col = self.cell(x, y)
        h, w = self.occ.shape
        inside = (row >= 0) & (row < h) & (col >= 0) & (col < w)
        out = np.ones(np.shape(row), dtype=bool)  # outside the map is wall
        out[inside] = self.occ[row[inside], col[inside]]
        return out

    def free(self, x, y) -> bool:
        return not bool(self.occupied(np.array([x]), np.array([y]))[0])


@dataclass
class Person:
    """Walks back and forth between two points at ``speed``."""

    a: tuple[float, float]
    b: tuple[float, float]
    speed: float = 1.2
    radius: float = 0.25
    phase: float = 0.0  # metres along the walk at t = 0

    def position(self, t: float) -> tuple[float, float]:
        length = math.dist(self.a, self.b)
        if length == 0:
            return self.a
        s = (self.phase + self.speed * t) % (2 * length)
        f = s / length if s <= length else 2 - s / length
        return self.a[0] + f * (self.b[0] - self.a[0]), self.a[1] + f * (self.b[1] - self.a[1])


@dataclass
class Box:
    """A full-height obstacle the map does not know about (bench, bin, sign board)."""

    x: float
    y: float
    half_x: float = 0.3
    half_y: float = 0.3


@dataclass
class Hit:
    distance: np.ndarray  # m along each ray (inf beyond max range)
    x: np.ndarray
    y: np.ndarray
    u: np.ndarray  # texture coordinate along the wall, m
    face: np.ndarray  # 0..3: which way the wall faces (texture variety)
    person: np.ndarray  # index of the person hit, -1 for a wall


@dataclass
class FloorWorld:
    floor: str
    grid: Grid
    people: list[Person] = field(default_factory=list)
    boxes: list[Box] = field(default_factory=list)

    def add_box(self, box: Box):
        """Burn a box into this floor's wall grid (the map Nav2 loads is a separate file)."""
        self.boxes.append(box)
        g = self.grid
        r0, c0 = g.cell(box.x - box.half_x, box.y + box.half_y)
        r1, c1 = g.cell(box.x + box.half_x, box.y - box.half_y)
        g.occ[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = True

    def people_at(self, t: float) -> list[tuple[float, float, float]]:
        return [(*p.position(t), p.radius) for p in self.people]

    def raycast(self, ox: float, oy: float, angles: np.ndarray, max_range: float = 15.0,
                t: float = 0.0, step: float = 0.025, people: bool = True) -> Hit:
        """First wall (or person) along each ray from (ox, oy) at world ``angles``."""
        g = self.grid
        angles = np.asarray(angles, dtype=float)
        dx, dy = np.cos(angles), np.sin(angles)
        ds = np.arange(step, max_range + step, step)
        xs = ox + dx[:, None] * ds[None, :]
        ys = oy + dy[:, None] * ds[None, :]
        occ = g.occupied(xs, ys)
        any_hit = occ.any(axis=1)
        k = np.where(any_hit, occ.argmax(axis=1), len(ds) - 1)
        # Bisect between the last free sample and the first wall sample.
        lo = np.where(k > 0, ds[np.maximum(k - 1, 0)], 0.0)
        hi = ds[k]
        for _ in range(5):
            mid = (lo + hi) / 2
            inwall = g.occupied(ox + dx * mid, oy + dy * mid)
            hi = np.where(inwall, mid, hi)
            lo = np.where(inwall, lo, mid)
        d = np.where(any_hit, hi, np.inf)
        # Which cell edge was crossed decides the wall's orientation.
        r_in, c_in = g.cell(ox + dx * lo, oy + dy * lo)
        r_w, c_w = g.cell(ox + dx * hi, oy + dy * hi)
        vertical = c_in != c_w  # crossed a column boundary: wall runs along y
        hx, hy = ox + dx * np.where(np.isfinite(d), d, max_range), oy + dy * np.where(np.isfinite(d), d, max_range)
        u = np.where(vertical, hy, hx)
        face = np.where(vertical, np.where(dx > 0, 0, 1), np.where(dy > 0, 2, 3))
        who = np.full(len(angles), -1)
        if people:
            for i, (px, py, r) in enumerate(self.people_at(t)):
                # Ray-circle intersection: |o + s d - p| = r.
                fx, fy = ox - px, oy - py
                b = fx * dx + fy * dy
                c = fx * fx + fy * fy - r * r
                disc = b * b - c
                s = -b - np.sqrt(np.maximum(disc, 0))
                hit = (disc >= 0) & (s > 0) & (s < d)
                d = np.where(hit, s, d)
                who = np.where(hit, i, who)
                ang = np.arctan2(oy + dy * s - py, ox + dx * s - px)
                u = np.where(hit, ang * r, u)
                face = np.where(hit, 4, face)
                hx = np.where(hit, ox + dx * s, hx)
                hy = np.where(hit, oy + dy * s, hy)
        return Hit(d, hx, hy, u, face, who)

    def collides(self, points: list[tuple[float, float]], t: float) -> bool:
        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])
        if self.grid.occupied(xs, ys).any():
            return True
        for px, py, r in self.people_at(t):
            if (np.hypot(xs - px, ys - py) < r).any():
                return True
        return False


class World:
    """All floors (loaded when first visited) and the floor the car is on."""

    def __init__(self, maps_dir: Path, floor: str):
        self.maps_dir = Path(maps_dir)
        self.floors: dict[str, FloorWorld] = {}
        self.floor = floor
        self.get(floor)

    def get(self, floor: str) -> FloorWorld:
        if floor not in self.floors:
            self.floors[floor] = FloorWorld(floor, Grid.load(self.maps_dir / "floors" / f"{floor}.yaml"))
        return self.floors[floor]

    @property
    def here(self) -> FloorWorld:
        return self.get(self.floor)
