"""PNG previews of a floor grid with the route graph drawn on top, for review."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .raster import GridSpec, rasterize

COLOURS = {
    "corridor": (0, 0, 0),
    "corridor_junction": (200, 0, 0),
    "door": (40, 120, 220),
    "doorway": (150, 0, 150),
    "stairs": (120, 120, 120),
    "elevator": (0, 160, 0),
    "entrance": (255, 140, 0),
}
EDGE_COLOURS = {"corridor": (30, 30, 160), "door": (150, 0, 150)}


def render_floor(floor, graph, spec: GridSpec, path: Path):
    fp = rasterize(floor.footprint, spec)
    free = rasterize(floor.free_space, spec)
    img = np.full(free.shape + (3,), 255, np.uint8)
    img[fp] = (215, 215, 215)
    img[free] = (245, 190, 110)
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)

    def px(n):
        c, r = spec.to_pixel(n["x"], n["y"])
        return float(c), float(r)

    fid = floor.id
    for e in graph.edges.values():
        a, b = graph.nodes[e["a"]], graph.nodes[e["b"]]
        if a["floor"] == fid and b["floor"] == fid:
            d.line([px(a), px(b)], fill=EDGE_COLOURS.get(e["type"], (0, 0, 0)), width=1)
    for n in graph.nodes.values():
        if n["floor"] != fid:
            continue
        x, y = px(n)
        r = {"corridor": 1, "door": 1}.get(n["type"], 3)
        d.ellipse([x - r, y - r, x + r, y + r], fill=COLOURS.get(n["type"], (0, 0, 0)))
    d.text((6, 6), f"{fid}  {floor.name}", fill=(0, 0, 0))
    path.parent.mkdir(parents=True, exist_ok=True)
    im.save(path, optimize=True)
