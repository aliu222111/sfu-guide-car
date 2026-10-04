"""How far along the route the car is, and how long is left (no ROS here).

The route is what the mission publishes on ``/guide/route``: graph nodes in
order, each ``{"id", "x", "y", "floor", "type"}`` in the map frame (metres).
Two consecutive nodes on the same floor are a stretch to drive; a change of
floor is an elevator ride, which costs time but no distance.
"""

from __future__ import annotations

import math


def _segments(route: list[dict]) -> list[tuple[dict, dict, float]]:
    out = []
    for a, b in zip(route, route[1:]):
        same = a.get("floor") == b.get("floor")
        out.append((a, b, math.hypot(b["x"] - a["x"], b["y"] - a["y"]) if same else 0.0))
    return out


def route_length(route: list[dict]) -> float:
    return sum(d for _, _, d in _segments(route))


def floor_changes(route: list[dict]) -> int:
    return sum(1 for a, b in zip(route, route[1:]) if a.get("floor") != b.get("floor"))


def remaining(route: list[dict], pose, floor: str | None) -> tuple[float, int] | None:
    """Metres and elevator rides left from the car's pose, or None if it cannot be placed.

    The pose is snapped to the nearest stretch of the route on the car's floor
    (the last such stretch when the route visits the floor more than once and
    the car is equally near both: it is further along).
    """
    segs = _segments(route)
    if not segs or pose is None:
        return None
    px, py = pose[0], pose[1]
    best = None
    for i, (a, b, d) in enumerate(segs):
        if a.get("floor") != b.get("floor") or a.get("floor") != floor:
            continue
        ax, ay, bx, by = a["x"], a["y"], b["x"], b["y"]
        t = 0.0 if d == 0 else max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / (d * d)))
        off = math.hypot(px - (ax + (bx - ax) * t), py - (ay + (by - ay) * t))
        if best is None or off <= best[0] + 1e-6:
            best = (off, i, t)
    if best is None:
        return None
    _, i, t = best
    left = segs[i][2] * (1.0 - t) + sum(d for _, _, d in segs[i + 1:])
    rides = sum(1 for a, b, _ in segs[i + 1:] if a.get("floor") != b.get("floor"))
    return left, rides


def estimate(route: list[dict], pose, floor: str | None, speed: float, ride_s: float = 60.0) -> dict | None:
    """``{"progress", "leftM", "etaSec"}`` for the whole route, or None."""
    total = route_length(route)
    r = remaining(route, pose, floor)
    if r is None or speed <= 0:
        return None
    left, rides = r
    return {
        "progress": 1.0 if total <= 0 else max(0.0, min(1.0, 1.0 - left / total)),
        "leftM": round(left, 1),
        "etaSec": round(left / speed + rides * ride_s, 1),
    }
