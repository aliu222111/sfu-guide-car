"""Spoken directions: turns along a path and the arrival message (spec section 8)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from sfu_map.route import RouteGraph

TURN_MIN = math.radians(40.0)  # smaller bends are followed without comment
ARM = 4.0  # metres before and after a node used to measure the bend


@dataclass
class Turn:
    node: str
    angle: float  # radians, positive = left (counter-clockwise)

    @property
    def direction(self) -> str:
        a = abs(self.angle)
        side = "left" if self.angle > 0 else "right"
        if a > math.radians(150):
            return "around"
        if a < math.radians(60):
            return f"slight {side}"
        return side


def _point_at(rg, path, i, dist, step):
    """Walk from node i along the path (step -1 or +1) until ``dist`` metres away."""
    x0, y0 = rg.nodes[path[i]]["x"], rg.nodes[path[i]]["y"]
    j = i
    while 0 <= j + step < len(path):
        j += step
        n = rg.nodes[path[j]]
        if n["floor"] != rg.nodes[path[i]]["floor"]:
            return None
        if math.hypot(n["x"] - x0, n["y"] - y0) >= dist:
            return n["x"], n["y"]
    return None


def find_turns(rg: RouteGraph, path: list[str]) -> list[Turn]:
    raw = []
    for i in range(1, len(path) - 1):
        a = _point_at(rg, path, i, ARM, -1)
        b = _point_at(rg, path, i, ARM, +1)
        if a is None or b is None:
            continue
        n = rg.nodes[path[i]]
        h1 = math.atan2(n["y"] - a[1], n["x"] - a[0])
        h2 = math.atan2(b[1] - n["y"], b[0] - n["x"])
        d = (h2 - h1 + math.pi) % (2 * math.pi) - math.pi
        if abs(d) >= TURN_MIN:
            raw.append((i, d))
    # One bend shows up at several neighbouring nodes; keep the sharpest.
    turns: list[Turn] = []
    group: list[tuple[int, float]] = []
    for item in raw:
        if group and (item[0] - group[-1][0] > 2 or math.copysign(1, item[1]) != math.copysign(1, group[-1][1])):
            i, d = max(group, key=lambda t: abs(t[1]))
            turns.append(Turn(path[i], d))
            group = []
        group.append(item)
    if group:
        i, d = max(group, key=lambda t: abs(t[1]))
        turns.append(Turn(path[i], d))
    return turns


def turn_text(t: Turn, distance: float) -> str:
    if t.direction == "around":
        return "Up ahead we turn around."
    metres = max(1, int(round(distance)))
    return f"In {metres} metres, turn {t.direction}."


def floor_label(rg: RouteGraph, floor: str | None) -> str:
    if floor is None:
        return "outside"
    meta = rg.floors.get(floor, {})
    building = meta.get("building") or floor.split("-")[0]
    name = meta.get("name") or floor
    return f"{building} {name}".replace(" Level", " level")


def arrival_text(rg: RouteGraph, path: list[str], destination: str) -> str:
    """Arrival message, with the side the room is on relative to the way we came."""
    goal = path[-1] if path else ""
    dest = None
    for key, d in rg.destinations.items():
        if d["node"] == goal:
            dest, destination = d, key
            break
    if dest is None or "room_x" not in dest or len(path) < 3:
        return f"We've arrived at {destination}."
    # Direction of travel: the last stretch of at least 1 m before the door.
    door = rg.nodes[goal]
    end = rg.nodes[path[-2]]
    start = None
    for nid in reversed(path[:-2]):
        n = rg.nodes[nid]
        if math.hypot(end["x"] - n["x"], end["y"] - n["y"]) >= 1.0:
            start = n
            break
    if start is None:
        return f"We've arrived at {destination}."
    hx, hy = end["x"] - start["x"], end["y"] - start["y"]
    rx, ry = dest["room_x"] - door["x"], dest["room_y"] - door["y"]
    cross = hx * ry - hy * rx
    dot = hx * rx + hy * ry
    if abs(cross) < abs(dot) * 0.5:
        side = "straight ahead" if dot > 0 else "behind you"
    else:
        side = "on your left" if cross > 0 else "on your right"
    return f"We've arrived. {destination} is {side}."
