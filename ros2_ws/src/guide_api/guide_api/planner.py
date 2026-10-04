"""Routes for a car that only drives straight: a few long legs with a turn between them (no ROS here).

Works on the Scout app's floor plan (``config/asb_plan.json``: a walkable grid
per level and the room labels), so it needs no built maps. A route is found on
the grid, keeping to the middle of halls, then pulled tight into the fewest
straight legs that stay clear of the walls. Standard library only.
"""

from __future__ import annotations

import base64
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path

from .frames import Affine

CLEAR = 2          # cells a straight leg must keep from any wall (2 cells is about 0.35 m)
SQRT2 = math.sqrt(2.0)


class PlanError(Exception):
    pass


@dataclass(frozen=True)
class Leg:
    """One straight drive, in the map frame: metres, heading in radians (0 east, counter-clockwise)."""

    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def length(self) -> float:
        return math.hypot(self.x1 - self.x0, self.y1 - self.y0)

    @property
    def heading(self) -> float:
        return math.atan2(self.y1 - self.y0, self.x1 - self.x0)


class Planner:
    def __init__(self, plan: Path | str, to_map: Affine):
        doc = json.loads(Path(plan).read_text())
        self.building = doc["building"]
        f = doc["frame"]
        self.cell, self.gw, self.gh = f["cell"], f["gw"], f["gh"]
        self.to_map = to_map
        self.to_plan = to_map.inverse()
        self._raw = {lv["id"]: lv["walk"] for lv in doc["levels"]}
        self._grid: dict[str, tuple[bytearray, bytearray]] = {}
        self.rooms = {(r["lv"], r["n"]): r for r in doc["rooms"]}
        self._dest = self.destinations()

    # --- rooms ---------------------------------------------------------------
    @staticmethod
    def level_of(floor: str) -> str:
        return floor.split("-", 1)[1].lstrip("0") or "0"

    def floor_of(self, level: str) -> str:
        return f"{self.building}-{int(level):02d}"

    def destinations(self) -> dict[str, dict]:
        """The rooms in the form of maps/destinations.json, for the service to check keys against."""
        out = {}
        for (lv, n), r in self.rooms.items():
            x, y = self.to_map.apply(r["x"], r["y"])
            out[f"{self.building} {n}"] = {"building": self.building, "room": n, "floor": self.floor_of(lv), "floor_name": f"{lv}000 Level",
                                           "type": r.get("k", ""), "name": "", "searchable": True, "room_x": round(x, 2), "room_y": round(y, 2)}
        return out

    def locate(self, key: str) -> tuple[str, float, float] | None:
        """"ASB 9896" -> (floor, x, y) in the map frame."""
        d = self._dest.get(key)
        return (d["floor"], d["room_x"], d["room_y"]) if d else None

    def nearest_room(self, floor: str, x: float, y: float, skip: str = "") -> str:
        """The room label nearest a map point, as a landmark to point the car at."""
        lv = self.level_of(floor)
        px, py = self.to_plan.apply(x, y)
        best, bd = "", float("inf")
        for (rl, n), r in self.rooms.items():
            if rl != lv or r.get("k") == "area" or n == skip:
                continue
            d = math.hypot(r["x"] - px, r["y"] - py)
            if d < bd:
                best, bd = n, d
        return f"{self.building} {best}" if best else ""

    # --- the grid --------------------------------------------------------------
    def grid(self, level: str) -> tuple[bytearray, bytearray]:
        """(walkable, clearance in cells capped at 20) for a level, built once."""
        if level in self._grid:
            return self._grid[level]
        if level not in self._raw:
            raise PlanError(f"no floor plan for level {level}")
        raw = base64.b64decode(self._raw[level])
        gw, gh = self.gw, self.gh
        n = gw * gh
        walk = bytearray(n)
        for i in range(n):
            walk[i] = (raw[i >> 3] >> (7 - (i & 7))) & 1
        clr = bytearray(n)
        for y in range(gh):
            row = y * gw
            for x in range(gw):
                i = row + x
                if walk[i]:
                    clr[i] = min(20, (clr[i - 1] if x else 0) + 1, (clr[i - gw] if y else 0) + 1)
        for y in range(gh - 1, -1, -1):
            row = y * gw
            for x in range(gw - 1, -1, -1):
                i = row + x
                if walk[i]:
                    clr[i] = min(clr[i], (clr[i + 1] if x < gw - 1 else 0) + 1, (clr[i + gw] if y < gh - 1 else 0) + 1)
        self._grid[level] = (walk, clr)
        return self._grid[level]

    def _snap(self, walk: bytearray, clr: bytearray, x: float, y: float) -> int:
        """The nearest walkable cell to a plan point, preferring one with room around it."""
        cx, cy = int(x // self.cell), int(y // self.cell)
        fallback = -1
        for r in range(0, 30):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.gw and 0 <= ny < self.gh:
                        i = ny * self.gw + nx
                        if walk[i]:
                            if clr[i] > CLEAR:
                                return i
                            if fallback < 0:
                                fallback = i
            if fallback >= 0 and r >= 6:
                return fallback
        return fallback

    def _dijkstra(self, walk: bytearray, clr: bytearray, start: int, goal: int) -> list[int]:
        gw, gh = self.gw, self.gh
        dist = {start: 0.0}
        prev: dict[int, int] = {}
        pq = [(0.0, start)]
        steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))
        while pq:
            d, u = heapq.heappop(pq)
            if u == goal:
                break
            if d > dist[u]:
                continue
            ux, uy = u % gw, u // gw
            for dx, dy, k in steps:
                vx, vy = ux + dx, uy + dy
                if vx < 0 or vy < 0 or vx >= gw or vy >= gh:
                    continue
                v = vy * gw + vx
                if not walk[v] or (dx and dy and (not walk[uy * gw + vx] or not walk[vy * gw + ux])):
                    continue
                c = clr[v]
                nd = d + k * (1.0 + 0.55 * max(0, 5 - c) + 0.06 * max(0, 20 - c))
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if goal != start and goal not in prev:
            raise PlanError("no way through on this level")
        cells = [goal]
        while cells[-1] != start:
            cells.append(prev[cells[-1]])
        cells.reverse()
        return cells

    def _clear_line(self, walk: bytearray, clr: bytearray, a: tuple[float, float], b: tuple[float, float]) -> bool:
        """True if the straight line a-b (in cells) stays CLEAR cells from every wall.

        Its two ends may be tighter (a doorway, a spot beside a wall): there it only has to be walkable."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(length * 2))
        for i in range(n + 1):
            t = i / n
            x, y = int(a[0] + (b[0] - a[0]) * t), int(a[1] + (b[1] - a[1]) * t)
            j = y * self.gw + x
            if not walk[j]:
                return False
            if clr[j] < CLEAR and min(t, 1 - t) * length > 5:
                return False
        return True

    def plan(self, floor: str, start: tuple[float, float], goal: tuple[float, float]) -> list[Leg]:
        """Straight legs from ``start`` to ``goal`` (map frame) on one floor."""
        level = self.level_of(floor)
        walk, clr = self.grid(level)
        s = self._snap(walk, clr, *self.to_plan.apply(*start))
        g = self._snap(walk, clr, *self.to_plan.apply(*goal))
        if s < 0 or g < 0:
            raise PlanError("that spot is not inside the building")
        cells = self._dijkstra(walk, clr, s, g)
        pts = [(c % self.gw + 0.5, c // self.gw + 0.5) for c in cells]
        # Pull the path tight: from each corner, jump to the furthest point still in clear sight.
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._clear_line(walk, clr, pts[i], pts[j]):
                j -= max(1, (j - i) // 8)
            out.append(pts[j])
            i = j
        legs = []
        for a, b in zip(out, out[1:]):
            ax, ay = self.to_map.apply(a[0] * self.cell, a[1] * self.cell)
            bx, by = self.to_map.apply(b[0] * self.cell, b[1] * self.cell)
            leg = Leg(ax, ay, bx, by)
            if leg.length > 0.05:
                legs.append(leg)
        return _merge(legs)


def _merge(legs: list[Leg], within: float = math.radians(8)) -> list[Leg]:
    """Join consecutive legs that point (almost) the same way: nobody wants to stop for a 5 degree turn."""
    out: list[Leg] = []
    for leg in legs:
        if out:
            d = abs((leg.heading - out[-1].heading + math.pi) % (2 * math.pi) - math.pi)
            if d < within:
                out[-1] = Leg(out[-1].x0, out[-1].y0, leg.x1, leg.y1)
                continue
        out.append(leg)
    return out


def turn_text(old: float | None, new: float) -> tuple[str, float]:
    """("left" | "right" | "around" | "ahead" | "point", degrees) for the turn from heading ``old`` to ``new``."""
    if old is None:
        return "point", 0.0
    d = math.degrees((new - old + math.pi) % (2 * math.pi) - math.pi)   # counter-clockwise positive: left
    if abs(d) < 12:
        return "ahead", d
    if abs(d) > 150:
        return "around", d
    return ("left" if d > 0 else "right"), d
