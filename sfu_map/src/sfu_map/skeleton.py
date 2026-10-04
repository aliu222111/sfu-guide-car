"""Corridor centrelines from the medial axis of the free-space mask.

The skeleton is traced into branches between junctions and endpoints, short
spurs are pruned, nearby junctions are merged, and each branch is resampled to
nodes no more than ``spacing`` metres apart (spec section 5).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from shapely.geometry import LineString
from shapely.geometry.base import BaseGeometry
from skimage.morphology import disk, skeletonize

from .raster import GridSpec, rasterize

_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Centrelines:
    nodes: list[tuple[float, float]]  # map-frame metres
    junction: list[bool]  # True for junctions and dead ends, False for spacing nodes
    edges: list[tuple[int, int]]


def centrelines(
    free: BaseGeometry,
    resolution: float = 0.05,  # the Nav2 grid's: a coarser raster can erase thin GIS walls
    spacing: float = 5.0,
    min_width: float = 0.45,  # the car is 0.38 m wide (nav2.yaml footprint); keep passages it fits
    spur_length: float = 2.5,
    merge_radius: float = 1.5,
) -> Centrelines:
    if free.is_empty:
        return Centrelines([], [], [])
    spec = GridSpec.around(free.bounds, resolution, margin=1.0)
    mask = rasterize(free, spec)
    # Drop slivers too narrow for the car before taking the medial axis.
    r = max(1, int(round(min_width / 2 / resolution)))
    mask = ndimage.binary_opening(mask, structure=disk(r))
    dist = ndimage.distance_transform_edt(mask) * resolution
    skel = skeletonize(mask)

    branches = _trace(skel)
    branches = _prune(branches, dist, resolution, spur_length)
    nodes, junction, edges = _resample(branches, spec, spacing, merge_radius)
    return Centrelines(nodes, junction, edges)


def _trace(skel: np.ndarray) -> list[list[tuple[int, int]]]:
    """Split a one-pixel skeleton into pixel paths between key pixels.

    Key pixels are those with other than two neighbours. Adjacent key pixels
    (the usual 2 to 4 pixel blob at a junction) share one path endpoint by being
    chained with zero-length steps, which ``_prune`` later contracts.
    """
    h, w = skel.shape
    pix = set(zip(*np.nonzero(skel)))

    def nbrs(p):
        r, c = p
        return [(r + dr, c + dc) for dr, dc in _NEIGHBOURS if (r + dr, c + dc) in pix]

    degree = {p: len(nbrs(p)) for p in pix}
    key = {p for p, d in degree.items() if d != 2}
    visited_steps: set[tuple] = set()
    branches = []

    def walk(start, first):
        path = [start, first]
        visited_steps.add((start, first))
        visited_steps.add((first, start))
        prev, cur = start, first
        while cur not in key:
            nxt = [n for n in nbrs(cur) if n != prev]
            if not nxt:
                break
            n = nxt[0]
            if (cur, n) in visited_steps:
                break
            visited_steps.add((cur, n))
            visited_steps.add((n, cur))
            path.append(n)
            prev, cur = cur, n
            if cur == start:
                break
        return path

    for k in key:
        for n in nbrs(k):
            if (k, n) not in visited_steps:
                branches.append(walk(k, n))
    # Closed loops with no key pixel at all.
    seen = {p for b in branches for p in b}
    for p in pix:
        if p in seen or degree[p] != 2:
            continue
        key.add(p)
        for n in nbrs(p):
            if (p, n) not in visited_steps:
                b = walk(p, n)
                branches.append(b)
                seen.update(b)
        key.discard(p)
    return branches


def _length(path, res):
    a = np.asarray(path, dtype=float)
    return float(np.sum(np.hypot(*np.diff(a, axis=0).T))) * res if len(a) > 1 else 0.0


def _prune(branches, dist, resolution, spur_length):
    """Contract key-pixel clusters into single vertices and remove short spurs."""
    # Union-find over key pixels joined by short (adjacent-pixel) branches.
    parent: dict = {}

    def find(p):
        parent.setdefault(p, p)
        while parent[p] != p:
            parent[p] = parent[parent[p]]
            p = parent[p]
        return p

    long_branches = []
    for b in branches:
        if len(b) == 2:
            parent[find(b[0])] = find(b[-1])
        else:
            long_branches.append(b)

    # Edges as (u, v, path) with cluster-representative endpoints.
    edges = []
    for b in long_branches:
        u, v = find(b[0]), find(b[-1])
        edges.append([u, v, b])

    changed = True
    while changed:
        changed = False
        deg: dict = {}
        for u, v, _ in edges:
            deg[u] = deg.get(u, 0) + 1
            deg[v] = deg.get(v, 0) + 1
        keep = []
        for e in edges:
            u, v, path = e
            if u == v and _length(path, resolution) < 2 * spur_length:
                changed = True  # small loop around a pillar or a GIS sliver
                continue
            leaf = deg[u] == 1 or deg[v] == 1
            if leaf and not (deg[u] == 1 and deg[v] == 1):
                hub = v if deg[u] == 1 else u
                if _length(path, resolution) < max(spur_length, 1.5 * dist[hub]):
                    changed = True
                    continue
            keep.append(e)
        edges = keep
        edges = _contract_degree_two(edges)
    return edges


def _contract_degree_two(edges):
    """Join edges through vertices of degree two so branches run key to key."""
    while True:
        incident: dict = {}
        for i, (u, v, _) in enumerate(edges):
            incident.setdefault(u, []).append(i)
            incident.setdefault(v, []).append(i)
        target = None
        for vtx, ids in incident.items():
            if len(ids) == 2 and ids[0] != ids[1]:
                target = (vtx, ids)
                break
        if target is None:
            return edges
        vtx, (i, j) = target
        a, b = edges[i], edges[j]
        pa = a[2] if a[1] == vtx else a[2][::-1]
        pb = b[2] if b[0] == vtx else b[2][::-1]
        ua = a[0] if a[1] == vtx else a[1]
        vb = b[1] if b[0] == vtx else b[0]
        merged = [ua, vb, pa + pb[1:]]
        edges = [e for k, e in enumerate(edges) if k not in (i, j)] + [merged]


def _resample(edges, spec: GridSpec, spacing: float, merge_radius: float):
    nodes: list[tuple[float, float]] = []
    junction: list[bool] = []
    index: dict = {}

    def vertex(p, path_end):
        if p not in index:
            x, y = spec.to_xy(path_end[1], path_end[0])
            index[p] = len(nodes)
            nodes.append((float(x), float(y)))
            junction.append(True)
        return index[p]

    out_edges = []
    for u, v, path in edges:
        a = np.asarray(path, dtype=float)
        xs, ys = spec.to_xy(a[:, 1], a[:, 0])
        line = LineString(np.column_stack([xs, ys]))
        iu = vertex(u, path[0])
        iv = vertex(v, path[-1])
        n = max(1, math.ceil(line.length / spacing))
        prev = iu
        for k in range(1, n):
            pt = line.interpolate(k / n, normalized=True)
            nodes.append((pt.x, pt.y))
            junction.append(False)
            cur = len(nodes) - 1
            out_edges.append((prev, cur))
            prev = cur
        if prev != iv:
            out_edges.append((prev, iv))
    return _merge_close(nodes, junction, out_edges, merge_radius)


def _merge_close(nodes, junction, edges, radius):
    """Merge junction vertices joined by an edge shorter than ``radius``."""
    parent = list(range(len(nodes)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in edges:
        if junction[a] and junction[b]:
            if math.dist(nodes[a], nodes[b]) < radius:
                parent[find(a)] = find(b)
    groups: dict[int, list[int]] = {}
    for i in range(len(nodes)):
        groups.setdefault(find(i), []).append(i)
    new_index = {}
    new_nodes, new_junction = [], []
    for root, members in groups.items():
        pts = np.asarray([nodes[m] for m in members])
        new_index.update({m: len(new_nodes) for m in members})
        new_nodes.append(tuple(pts.mean(axis=0).tolist()))
        new_junction.append(any(junction[m] for m in members))
    new_edges = set()
    for a, b in edges:
        a, b = new_index[a], new_index[b]
        if a != b:
            new_edges.add((min(a, b), max(a, b)))
    return new_nodes, new_junction, sorted(new_edges)
