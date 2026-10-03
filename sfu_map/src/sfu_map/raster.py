"""Rasterise map-frame polygons and write Nav2 map_server images."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from shapely.geometry import MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry

# Nav2 trinary pixel values with negate: 0.
FREE = 254
OCCUPIED = 0


@dataclass(frozen=True)
class GridSpec:
    """Axis-aligned grid in the map frame. Row 0 is the northern edge."""

    origin_x: float  # x of the western edge
    origin_y: float  # y of the southern edge
    resolution: float
    width: int
    height: int

    @classmethod
    def around(cls, bounds, resolution: float, margin: float = 1.0) -> "GridSpec":
        minx, miny, maxx, maxy = bounds
        # Snap the origin to the resolution so grids of one building line up.
        ox = math.floor((minx - margin) / resolution) * resolution
        oy = math.floor((miny - margin) / resolution) * resolution
        w = int(math.ceil((maxx + margin - ox) / resolution))
        h = int(math.ceil((maxy + margin - oy) / resolution))
        return cls(round(ox, 6), round(oy, 6), resolution, w, h)

    def to_pixel(self, x, y):
        """Map metres to fractional (column, row) image coordinates."""
        col = (np.asarray(x) - self.origin_x) / self.resolution
        row = self.height - (np.asarray(y) - self.origin_y) / self.resolution
        return col, row

    def to_xy(self, col, row):
        """Pixel centre to map metres."""
        x = self.origin_x + (np.asarray(col) + 0.5) * self.resolution
        y = self.origin_y + (self.height - np.asarray(row) - 0.5) * self.resolution
        return x, y


def _polygons(geom: BaseGeometry):
    if isinstance(geom, Polygon):
        yield geom
    elif isinstance(geom, MultiPolygon) or hasattr(geom, "geoms"):
        for g in geom.geoms:
            yield from _polygons(g)


def rasterize(geom: BaseGeometry, spec: GridSpec) -> np.ndarray:
    """Boolean mask, True inside ``geom``.

    Polygons are drawn largest first, each exterior then its holes, so an island
    inside another polygon's hole is drawn after the hole and survives.
    """
    img = Image.new("1", (spec.width, spec.height), 0)
    draw = ImageDraw.Draw(img)

    def ring(coords):
        c, r = spec.to_pixel(*np.asarray(coords).T[:2])
        return list(zip(c.tolist(), r.tolist()))

    for poly in sorted(_polygons(geom), key=lambda p: -p.area):
        if poly.is_empty:
            continue
        draw.polygon(ring(poly.exterior.coords), fill=1)
        for hole in poly.interiors:
            draw.polygon(ring(hole.coords), fill=0)
    return np.array(img, dtype=bool)


def write_nav2_map(mask: np.ndarray, spec: GridSpec, image_path: Path) -> Path:
    """Write ``image_path`` (PNG or PGM) and its map_server YAML beside it.

    Returns the YAML path.
    """
    image_path = Path(image_path)
    image_path.parent.mkdir(parents=True, exist_ok=True)
    pixels = np.where(mask, FREE, OCCUPIED).astype(np.uint8)
    Image.fromarray(pixels, mode="L").save(image_path, optimize=True)
    meta = {
        "image": image_path.name,
        "mode": "trinary",
        "resolution": spec.resolution,
        "origin": [spec.origin_x, spec.origin_y, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path = image_path.with_suffix(".yaml")
    with open(yaml_path, "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False, default_flow_style=None)
    return yaml_path


def load_nav2_map(yaml_path: Path) -> tuple[np.ndarray, GridSpec]:
    """Read a map the way nav2_map_server does (trinary, negate 0). Mask is True where free."""
    yaml_path = Path(yaml_path)
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = np.array(Image.open(yaml_path.parent / meta["image"]).convert("L"), dtype=float)
    occ = (255.0 - img) / 255.0
    free = occ < meta["free_thresh"]
    h, w = img.shape
    ox, oy, _ = meta["origin"]
    return free, GridSpec(ox, oy, meta["resolution"], w, h)
