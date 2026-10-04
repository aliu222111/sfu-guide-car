"""Where on a floor the car fits, from the floor's occupancy grid (sfu_map).

A position estimate inside a wall is impossible, and Nav2 cannot plan from it
("Start occupied": the 75 cm car's footprint overlaps the wall). A fix that
lands there (a sign read from a wrong heading, a door number spoken by the
student: the door is in the wall line) is moved to the nearest place the car
fits.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np


class FloorSpace:
    def __init__(self, free: np.ndarray, resolution: float, origin: tuple[float, float]):
        import cv2

        self.res = resolution
        self.ox, self.oy = origin
        self.h, self.w = free.shape
        self.clear = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5) * resolution  # m to the nearest wall

    @classmethod
    def load(cls, yaml_path: Path) -> "FloorSpace":
        import cv2
        import yaml

        meta = yaml.safe_load(Path(yaml_path).read_text())
        img = cv2.imread(str(Path(yaml_path).with_name(meta["image"])), cv2.IMREAD_GRAYSCALE)
        occ = (255 - img.astype(np.float32)) / 255.0 if not meta.get("negate") else img / 255.0
        return cls(occ < meta.get("free_thresh", 0.25), meta["resolution"], tuple(meta["origin"][:2]))

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return self.h - 1 - int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def clearance(self, x: float, y: float) -> float:
        r, c = self._cell(x, y)
        if not (0 <= r < self.h and 0 <= c < self.w):
            return 0.0
        return float(self.clear[r, c])

    def nearest_fit(self, x: float, y: float, clearance: float = 0.4, search: float = 4.0) -> tuple[float, float] | None:
        """The nearest point within ``search`` m with at least ``clearance`` m to any wall
        (or the most open point there, if none is that open); None if all is wall."""
        r, c = self._cell(x, y)
        k = int(search / self.res)
        r0, r1, c0, c1 = max(0, r - k), min(self.h, r + k + 1), max(0, c - k), min(self.w, c + k + 1)
        win = self.clear[r0:r1, c0:c1]
        if win.size == 0 or win.max() <= 0:
            return None
        rr, cc = np.mgrid[r0:r1, c0:c1]
        dist = np.hypot(rr - r, cc - c)
        need = min(clearance, float(win.max()))
        dist[win < need] = np.inf
        i = np.unravel_index(int(np.argmin(dist)), dist.shape)
        row, col = rr[i], cc[i]
        return self.ox + (col + 0.5) * self.res, self.oy + (self.h - row - 0.5) * self.res
