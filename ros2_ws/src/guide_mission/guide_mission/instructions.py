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


KEEP_TYPES = {"corridor_junction", "door", "doorway", "elevator", "entrance", "outdoor_junction"}


def waypoints(rg: RouteGraph, nodes: list[str], min_gap: float = 6.0, max_gap: float = 20.0) -> list[str]:
    """The nodes of a leg worth sending to Nav2 as poses.

    Nav2 drives through every pose it is given; passing within a corner node's
    radius is not guaranteed, and a pose left behind makes the car turn back.
    The grid planner follows corridors by itself, so keep only junctions, turns
    and transitions at least ``min_gap`` apart, a pose at least every
    ``max_gap`` metres so the route between them cannot change, and the goal.
    """
    if len(nodes) <= 2:
        return list(nodes)
    turn_nodes = {t.node for t in find_turns(rg, nodes)}
    kept = [nodes[0]]
    since = 0.0
    for prev, nid in zip(nodes, nodes[1:-1]):
        a, b = rg.nodes[prev], rg.nodes[nid]
        since += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        last = rg.nodes[kept[-1]]
        gap = math.hypot(b["x"] - last["x"], b["y"] - last["y"])
        key = nid in turn_nodes or b["type"] in KEEP_TYPES
        if (key and gap >= min_gap) or since >= max_gap:
            kept.append(nid)
            since = 0.0
    kept.append(nodes[-1])
    return kept


def corridor_axis(rg: RouteGraph, nodes: list[str], progress: int, yaw: float | None,
                  max_dev: float = 0.5, min_len: float = 4.0, align_tol: float = math.radians(15.0)) -> float | None:
    """Direction of the corridor the car is driving along, for the vanishing-point heading.

    The straight run of route nodes from the last one reached: extend while
    every node in between stays within ``max_dev`` of the line (graph nodes on
    a skeleton wobble, and one short edge can be 20 degrees off the corridor).
    None when the run is shorter than ``min_len`` or the car is not lined up
    with it (``align_tol``, either way along): in a turn, or before one, the
    vanishing point belongs to a different corridor than the route's next edge
    (found in the HIL simulation, where it dragged the heading 20 degrees at a time).
    """
    if progress >= len(nodes) - 1:
        return None
    pts = [(rg.nodes[n]["x"], rg.nodes[n]["y"]) for n in nodes[progress:]]
    best = None
    for j in range(1, len(pts)):
        (ax, ay), (bx, by) = pts[0], pts[j]
        length = math.hypot(bx - ax, by - ay)
        if length < 1e-6:
            continue
        ok = all(abs((bx - ax) * (py - ay) - (by - ay) * (px - ax)) / length <= max_dev for px, py in pts[1:j])
        if not ok:
            break
        best = (length, math.atan2(by - ay, bx - ax))
    if best is None or best[0] < min_len:
        return None
    axis = best[1]
    if yaw is not None:
        d = (yaw - axis + math.pi) % (2 * math.pi) - math.pi
        if min(abs(d), math.pi - abs(d)) > align_tol:
            return None
    return axis
