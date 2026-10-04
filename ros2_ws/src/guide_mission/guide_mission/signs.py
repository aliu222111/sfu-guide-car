"""Room-sign reads as absolute position fixes (spec section 6, "Absolute fixes from signs")."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

from sfu_map.route import RouteGraph

MAX_DISTANCE = 8.0  # spec: only when the current estimate is within 8 m of that door
FIX_SIGMA = 3.0  # m: reading distance and angle are unknown; looser so one read nudges, not yanks
READ_DISTANCE = 2.5  # m: how far ahead a room sign is typically read (640 px camera, 5 cm digits)

_READ = re.compile(r"^(?:(AQ|WMC|MBC)\s*)?(\d{3,4}[A-Z]?(?:\.\d+)?)$")


@dataclass
class SignFix:
    room: str
    node: str
    x: float
    y: float
    sigma: float
    distance: float  # from the current estimate


def match_sign(rg: RouteGraph, text: str, floor: str | None, x: float | None, y: float | None, lost: bool = False) -> SignFix | None:
    """A fix if ``text`` names a door on ``floor`` near the current estimate.

    A bare number is taken to be in the current floor's building. When ``lost``
    (after an elevator ride) the distance check is skipped: any door on the
    floor matching the read is accepted.
    """
    if floor is None:
        return None
    m = _READ.match(text.strip().upper())
    if not m:
        return None
    building = m.group(1) or floor.split("-")[0]
    key = f"{building} {m.group(2)}"
    d = rg.destinations.get(key)
    if d is None or d["floor"] != floor:
        return None
    n = rg.nodes[d["node"]]
    dist = math.inf if x is None or y is None else math.hypot(n["x"] - x, n["y"] - y)
    if not lost and dist > MAX_DISTANCE:
        return None
    return SignFix(key, d["node"], n["x"], n["y"], FIX_SIGMA, dist)


def viewpoint(door_x: float, door_y: float, yaw: float, bearing: float = 0.0,
              distance: float = READ_DISTANCE) -> tuple[float, float]:
    """Where the car is when it reads the sign at a door.

    The camera reads signs ahead of the car, not beside it: placing the car at
    the door would pull the estimate forward by the reading distance at every
    sign. Step back from the door along the line of sight (heading plus the
    sign's bearing in the image, 0 when the reader gives no box).
    """
    a = yaw + bearing
    return door_x - distance * math.cos(a), door_y - distance * math.sin(a)
