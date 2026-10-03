"""Room-sign reads as absolute position fixes (spec section 6, "Absolute fixes from signs")."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

from sfu_map.route import RouteGraph

MAX_DISTANCE = 8.0  # spec: only when the current estimate is within 8 m of that door
FIX_SIGMA = 2.0  # the car reads a sign from up to a few metres away

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
