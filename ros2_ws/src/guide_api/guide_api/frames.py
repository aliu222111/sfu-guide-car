"""The rider app's floor-plan coordinates <-> the car's map frame (no ROS here).

The app draws a building from its SFU keyplan drawing, in the drawing's own
units (x right, y down). The car works in the map frame of ``maps/route_graph.json``
(east, north, metres). One affine transform per building joins them; it is
fitted from rooms present in both (``python -m guide_api.fit_frame``) and kept
in ``guide_api/config/app_frames.json``.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Affine:
    """x' = a*x + b*y + c ;  y' = d*x + e*y + f"""

    a: float
    b: float
    c: float
    d: float
    e: float
    f: float

    def apply(self, x: float, y: float) -> tuple[float, float]:
        return (self.a * x + self.b * y + self.c, self.d * x + self.e * y + self.f)

    def inverse(self) -> "Affine":
        det = self.a * self.e - self.b * self.d
        if abs(det) < 1e-12:
            raise ValueError("transform cannot be inverted")
        a, b, d, e = self.e / det, -self.b / det, -self.d / det, self.a / det
        return Affine(a, b, -(a * self.c + b * self.f), d, e, -(d * self.c + e * self.f))


def fit(pairs: list[tuple[tuple[float, float], tuple[float, float]]]) -> Affine:
    """Least-squares affine taking each pair's first point to its second (needs 3 or more)."""
    if len(pairs) < 3:
        raise ValueError("need at least three point pairs")
    # Normal equations for [x y 1] * [p q r]^T = target, once for x' and once for y'.
    sxx = sxy = sx = syy = sy = n = 0.0
    tx = [0.0, 0.0, 0.0]
    ty = [0.0, 0.0, 0.0]
    for (x, y), (u, v) in pairs:
        sxx += x * x; sxy += x * y; sx += x; syy += y * y; sy += y; n += 1
        tx[0] += x * u; tx[1] += y * u; tx[2] += u
        ty[0] += x * v; ty[1] += y * v; ty[2] += v
    m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]

    def solve(rhs):
        a = [row[:] + [r] for row, r in zip(m, rhs)]
        for i in range(3):
            p = max(range(i, 3), key=lambda k: abs(a[k][i]))
            if abs(a[p][i]) < 1e-12:
                raise ValueError("points are collinear")
            a[i], a[p] = a[p], a[i]
            for k in range(3):
                if k != i:
                    w = a[k][i] / a[i][i]
                    a[k] = [vk - w * vi for vk, vi in zip(a[k], a[i])]
        return [a[i][3] / a[i][i] for i in range(3)]

    p, q = solve(tx), solve(ty)
    return Affine(p[0], p[1], p[2], q[0], q[1], q[2])


class AppFrames:
    """Per-building transforms, and the app's level id for a map floor id ("ASB-09" -> "9")."""

    def __init__(self, to_map: dict[str, Affine] | None = None):
        self.to_map = dict(to_map or {})
        self.to_app = {b: t.inverse() for b, t in self.to_map.items()}

    @classmethod
    def load(cls, path: Path | str) -> "AppFrames":
        path = Path(path)
        if not path.is_file():
            return cls()
        doc = json.loads(path.read_text())
        return cls({b: Affine(*v["keyplan_to_map"]) for b, v in doc.get("buildings", {}).items()})

    @staticmethod
    def split_floor(floor: str | None) -> tuple[str, str] | None:
        """"ASB-09" -> ("ASB", "9"); None for outdoors or an odd id."""
        if not floor or "-" not in floor:
            return None
        building, level = floor.split("-", 1)
        return building, (level.lstrip("0") or "0")

    def point(self, floor: str | None, x: float, y: float) -> dict | None:
        """A map-frame point as the app wants it: ``{"lv", "x", "y"}``, or None if the building has no transform."""
        sf = self.split_floor(floor)
        if sf is None or sf[0] not in self.to_app:
            return None
        ax, ay = self.to_app[sf[0]].apply(x, y)
        return {"lv": sf[1], "x": round(ax, 1), "y": round(ay, 1)}
