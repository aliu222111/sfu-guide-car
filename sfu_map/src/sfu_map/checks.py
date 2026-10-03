"""Consistency checks between the route graph and the Nav2 grids."""

from __future__ import annotations

from collections import Counter

import numpy as np
from scipy import ndimage

from .raster import GridSpec

# The car's half-width (footprint in guide_bringup/config/nav2.yaml): Nav2's
# planner cannot put the car's centre closer than this to a wall.
INSCRIBED_RADIUS = 0.19


def grid_splits(graph, floor: str, free: np.ndarray, spec: GridSpec, radius: float = INSCRIBED_RADIUS) -> list[str]:
    """Graph pieces on ``floor`` that the grid splits for a car of ``radius``.

    Returns one message per same-floor graph piece whose nodes fall in more
    than one connected region of the grid once walls are grown by ``radius``:
    the graph promises a route Nav2 cannot drive.
    """
    clear = ndimage.distance_transform_edt(free) * spec.resolution > radius
    regions, _ = ndimage.label(clear)
    parent = {nid: nid for nid, n in graph.nodes.items() if n["floor"] == floor}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for e in graph.edges.values():
        if e["a"] in parent and e["b"] in parent and e["type"] == "corridor":
            parent[find(e["a"])] = find(e["b"])
    pieces: dict[str, list[str]] = {}
    for nid in parent:
        pieces.setdefault(find(nid), []).append(nid)
    out = []
    for members in pieces.values():
        labels = Counter()
        for nid in members:
            n = graph.nodes[nid]
            c, r = spec.to_pixel(n["x"], n["y"])
            r, c = int(r), int(c)
            if 0 <= r < clear.shape[0] and 0 <= c < clear.shape[1] and regions[r, c]:
                labels[regions[r, c]] += 1
        if len(labels) > 1:
            out.append(f"{floor}: a graph piece of {len(members)} nodes spans {len(labels)} grid regions {sorted(labels.values(), reverse=True)[:4]}")
    return out


def node_regions(graph, floor: str, free: np.ndarray, spec: GridSpec, radius: float = INSCRIBED_RADIUS, snap: float = 0.4) -> dict[str, int]:
    """Grid region of every node on ``floor`` (0 = none within ``snap`` metres)."""
    clear = ndimage.distance_transform_edt(free) * spec.resolution > radius
    regions, _ = ndimage.label(clear)
    # For nodes just inside the grown walls (a door in a narrow corridor), use
    # the nearest clear cell.
    dist, (ri, ci) = ndimage.distance_transform_edt(regions == 0, return_indices=True)
    out = {}
    for nid, n in graph.nodes.items():
        if n["floor"] != floor:
            continue
        c, r = spec.to_pixel(n["x"], n["y"])
        r, c = int(r), int(c)
        if not (0 <= r < clear.shape[0] and 0 <= c < clear.shape[1]):
            out[nid] = 0
        elif dist[r, c] * spec.resolution <= snap:
            out[nid] = int(regions[ri[r, c], ci[r, c]])
        else:
            out[nid] = 0
    return out


def prune_to_grid(graph, floor: str, free: np.ndarray, spec: GridSpec, radius: float = INSCRIBED_RADIUS) -> list[str]:
    """Remove same-floor edges whose ends the car cannot drive between on the grid.

    Thin GIS walls (a few centimetres) can vanish when polygons are merged or
    a line of sight is tested with a small tolerance; the Nav2 grid keeps
    them. Returns a message per removed edge.
    """
    labels = node_regions(graph, floor, free, spec, radius)
    removed = []
    for key, e in list(graph.edges.items()):
        a, b = e["a"], e["b"]
        if a not in labels or b not in labels or e["type"] not in ("corridor", "door"):
            continue
        la, lb = labels[a], labels[b]
        if e["type"] == "door":
            continue  # doorways cross a GIS wall line on purpose; verified on site
        if la != lb or la == 0:
            del graph.edges[key]
            removed.append(f"{floor}: {a} - {b} ({e['length']:.1f} m) crosses a wall or a gap narrower than the car")
    return removed
