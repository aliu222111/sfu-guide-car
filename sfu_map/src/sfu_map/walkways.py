"""Accessible walkways from SFU's 2024 accessibility map (spec section 5, "Outdoor footpaths").

The OSM footpath pull still fails, so the outdoor network comes from
``data/sfu_pdf/sfu_Accessibility_campus_map_2024.pdf``: its covered (dashed)
and uncovered (solid) step-free walkways are drawn in one navy colour, with
labels such as "AQ 3000" where a walkway enters a building at that level.

    sfu-map walkways      # needs pymupdf; writes data/sfu_derived/accessible_walkways.geojson

1. Render the map page and keep navy pixels, minus text, the legend and the
   compact icons (door openers, entrances, parking badges).
2. Georeference: a 2D similarity from PDF points to the map frame, started
   from building-name labels and refined by maximising the overlap of the
   PDF's building fills with the GIS footprints.
3. Resample into the map frame, join the dashes, take the skeleton and trace
   it into a walkway network.
4. Where a walkway crosses the footprint of AQ, MBC or WMC, emit an entrance
   on the floor named by the nearest level label.

The output is committed, so ``sfu-map build`` does not need pymupdf. Every
walkway and entrance is ``verified: false`` until checked on site.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import ndimage, optimize
from shapely.geometry import LineString, MultiLineString, Point, mapping
from shapely.ops import linemerge, unary_union
from skimage.morphology import disk, skeletonize

from .geo import xy_to_lonlat
from .gisdata import REPO_ROOT, load_building_footprints, load_floors
from .raster import GridSpec, rasterize
from .skeleton import _prune, _trace

PDF = REPO_ROOT / "data" / "sfu_pdf" / "sfu_Accessibility_campus_map_2024.pdf"
OUT = REPO_ROOT / "data" / "sfu_derived" / "accessible_walkways.geojson"

DPI = 300
NAVY = np.array([60, 85, 165])
NAVY_TOL = 45
# Building fills on this map (RGB quantised to multiples of 8).
BUILDING_FILLS = [(168, 192, 184), (176, 192, 184), (160, 184, 184), (168, 184, 184), (72, 160, 160), (88, 168, 168), (112, 176, 168)]
# Building-name labels used for the first georeferencing guess.
NAME_LABELS = {
    "SH": ["Strand"], "SWH": ["Saywell"], "EDB": ["Education"], "LDC": ["Lorne", "Davies"],
    "TC": ["Transportation"], "RCB": ["Robert", "C."], "WMC": ["West", "Mall"],
}
MAPPED = ("AQ", "MBC", "WMC")  # buildings with indoor floor data
GRID = 0.5  # m, map-frame raster for tracing
OPEN_PX = 4
DASH_GAP_PX = 30  # px at DPI: longer than the gap between two dashes
DASH_JOIN = 3.0  # m, final closing in the map frame: joins corners and gaps at icons
LABEL_RADIUS = 45.0  # m, a level label this close to a crossing names its floor
ENTRANCE_REACH = 25.0  # m, that floor must have corridor this close to the crossing
LABEL_FALLBACK = 150.0  # m, a crossing with no label nearby takes the building's nearest one


@dataclass
class Similarity:
    scale: float
    yaw: float
    tx: float
    ty: float

    def pdf_to_map(self, px, py):
        """PDF points (y down) to map metres."""
        x, y = np.asarray(px, float), -np.asarray(py, float)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.scale * (c * x - s * y) + self.tx, self.scale * (s * x + c * y) + self.ty

    def map_to_pdf(self, mx, my):
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        x, y = (np.asarray(mx, float) - self.tx) / self.scale, (np.asarray(my, float) - self.ty) / self.scale
        return c * x - s * y, -(s * x + c * y)


def _render(pdf: Path):
    import pymupdf

    page = pymupdf.open(str(pdf))[0]
    pix = page.get_pixmap(dpi=DPI)
    img = np.frombuffer(pix.samples, np.uint8).reshape(pix.h, pix.w, pix.n)[:, :, :3].astype(int)
    rot = page.rotation_matrix
    words = [(pymupdf.Rect(w[:4]) * rot, w[4], w[5], w[6], w[7]) for w in page.get_text("words")]
    return img, words, page.rect


def _walkway_mask(img, words, page_rect) -> np.ndarray:
    k = DPI / 72
    m = np.abs(img - NAVY).sum(2) < NAVY_TOL
    # The legend, index and title blocks sit outside the campus drawing.
    h, w = m.shape
    m[int(0.47 * h):, int(0.79 * w):] = False
    m[int(0.83 * h):, :] = False
    m[: int(0.22 * h), : int(0.36 * w)] = False
    # Walkways are drawn at least 4 px thick; ring icons, arrows and text
    # strokes are under 3 px. An opening with a 4 px square keeps only the
    # walkways and the solid badges (parking "P", entrance triangles).
    m = ndimage.binary_opening(m, structure=np.ones((OPEN_PX, OPEN_PX)))
    lab, n = ndimage.label(m, structure=np.ones((3, 3)))
    keep = np.ones(n + 1, bool)
    keep[0] = False
    for i, sl in enumerate(ndimage.find_objects(lab), start=1):
        h, w = sl[0].stop - sl[0].start, sl[1].stop - sl[1].start
        fill = (lab[sl] == i).sum() / (h * w)
        if min(h, w) > 12 and fill > 0.45:
            keep[i] = False  # a solid badge, not a line or a corner
    m = keep[lab]
    # Join the dashes along their own direction. Walkways run along the PDF's
    # axes, so line-shaped closings join a dashed line without merging two
    # parallel walkways.
    joined = m.copy()
    for se in (np.ones((1, DASH_GAP_PX)), np.ones((DASH_GAP_PX, 1))):
        joined |= ndimage.binary_closing(m, structure=se)
    return joined


def _find_label(words, seq, max_y):
    for w in words:
        if w[1] != seq[0] or w[0].y1 > max_y:
            continue
        line = sorted([x for x in words if x[2] == w[2] and x[3] == w[3]], key=lambda x: x[4])
        i = line.index(w)
        if [x[1] for x in line[i: i + len(seq)]] == seq:
            r = line[i][0]
            for q in line[i + 1: i + len(seq)]:
                r = r | q[0]
            return (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2
    return None


def _solve_similarity(src, dst):
    """Least-squares 2D similarity (Umeyama) taking ``src`` onto ``dst``."""
    a, b = np.asarray(src, float), np.asarray(dst, float)
    ma, mb = a.mean(0), b.mean(0)
    A, B = a - ma, b - mb
    U, S, Vt = np.linalg.svd(B.T @ A / len(a))
    D = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[1, 1] = -1
    R = U @ D @ Vt
    scale = float(np.trace(np.diag(S) @ D) / ((A**2).sum() / len(a)))
    t = mb - scale * R @ ma
    return scale, math.atan2(R[1, 0], R[0, 0]), float(t[0]), float(t[1])


def _building_mask(img) -> np.ndarray:
    q = img // 8 * 8
    m = np.zeros(img.shape[:2], bool)
    for c in BUILDING_FILLS:
        m |= (q == c).all(2)
    return m


def georeference(img, words, page_rect, footprints) -> tuple[Similarity, float]:
    """Similarity from PDF points to the map frame, and the building overlap (IoU) it reaches."""
    max_y = page_rect.height * 0.82  # skip the building index under the map
    src, dst = [], []
    for code, seq in NAME_LABELS.items():
        c = _find_label(words, seq, max_y)
        if c and code in footprints:
            src.append((c[0], -c[1]))
            dst.append(footprints[code].centroid.coords[0])
    if len(src) < 3:
        raise RuntimeError("not enough building labels found to georeference the map")
    sim = Similarity(*_solve_similarity(src, dst))

    pdf_b = _building_mask(img)
    spec = GridSpec.around((-900, -500, 700, 500), 2.0, 0)
    gis = rasterize(unary_union(list(footprints.values())), spec)
    cc, rr = np.meshgrid(np.arange(gis.shape[1]), np.arange(gis.shape[0]))
    mx, my = spec.to_xy(cc, rr)
    k = DPI / 72

    def iou(v):
        sm = Similarity(*v)
        px, py = sm.map_to_pdf(mx, my)
        u, w = (px * k).astype(int), (py * k).astype(int)
        ok = (u >= 0) & (u < pdf_b.shape[1]) & (w >= 0) & (w < pdf_b.shape[0])
        pdf = np.zeros_like(gis)
        pdf[ok] = pdf_b[w[ok], u[ok]]
        return (pdf & gis).sum() / max((pdf | gis).sum(), 1)

    best = [sim.scale, sim.yaw, sim.tx, sim.ty]
    best_v = iou(best)
    for dyaw in np.radians(np.arange(-4, 4.1, 1)):
        for ds in (0.97, 1.0, 1.03):
            for dx in range(-40, 41, 10):
                for dy in range(-40, 41, 10):
                    cand = [sim.scale * ds, sim.yaw + dyaw, sim.tx + dx, sim.ty + dy]
                    v = iou(cand)
                    if v > best_v:
                        best, best_v = cand, v
    simplex = [best] + [[b + (d if i == j else 0) for j, b in enumerate(best)] for i, d in enumerate((best[0] * 0.01, 0.005, 3.0, 3.0))]
    res = optimize.minimize(lambda v: -iou(v), best, method="Nelder-Mead",
                            options={"initial_simplex": simplex, "xatol": 1e-4, "fatol": 1e-5, "maxiter": 600})
    return Similarity(*res.x), -res.fun


def _to_map_grid(mask, sim: Similarity, spec: GridSpec) -> np.ndarray:
    cc, rr = np.meshgrid(np.arange(spec.width), np.arange(spec.height))
    mx, my = spec.to_xy(cc, rr)
    px, py = sim.map_to_pdf(mx, my)
    k = DPI / 72
    u, w = (px * k).astype(int), (py * k).astype(int)
    ok = (u >= 0) & (u < mask.shape[1]) & (w >= 0) & (w < mask.shape[0])
    out = np.zeros((spec.height, spec.width), bool)
    out[ok] = mask[w[ok], u[ok]]
    return out


def trace_network(grid: np.ndarray, spec: GridSpec, spur: float = 8.0) -> list[LineString]:
    """Walkway centrelines from a map-frame mask, as map-frame polylines."""
    joined = ndimage.binary_closing(grid, structure=disk(int(round(DASH_JOIN / spec.resolution))))
    joined = ndimage.binary_opening(joined, structure=disk(1))
    skel = skeletonize(joined)
    dist = ndimage.distance_transform_edt(joined) * spec.resolution
    edges = _prune(_trace(skel), dist, spec.resolution, spur)
    lines = []
    for _, _, path in edges:
        a = np.asarray(path, float)
        if len(a) < 2:
            continue
        xs, ys = spec.to_xy(a[:, 1], a[:, 0])
        line = LineString(np.column_stack([xs, ys])).simplify(1.0)
        if line.length >= 2.0:
            lines.append(line)
    lines = bridge_gaps(lines)
    merged = linemerge(unary_union(lines))
    return list(getattr(merged, "geoms", [merged]))


SNAP = 40.0  # m: icons drawn on top of a walkway (two parking badges at TC) leave gaps up to this
SNAP_ANGLE = math.radians(30)


def bridge_gaps(lines: list[LineString]) -> list[LineString]:
    """Join a dangling walkway end to the next walkway it points at.

    An end counts as dangling when no other line touches it. It is joined to
    the nearest point of another line within ``SNAP`` metres, if that point
    lies roughly ahead (within ``SNAP_ANGLE`` of the line's direction).
    """
    out = list(lines)
    for i, line in enumerate(lines):
        coords = list(line.coords)
        for end, prev in ((coords[0], coords[min(1, len(coords) - 1)]), (coords[-1], coords[max(-2, -len(coords))])):
            p = Point(end)
            others = [l for j, l in enumerate(lines) if j != i]
            if any(o.distance(p) < 1.0 for o in others):
                continue
            heading = math.atan2(end[1] - prev[1], end[0] - prev[0])
            best = None
            for o in others:
                d = o.distance(p)
                if d > SNAP:
                    continue
                q = o.interpolate(o.project(p))
                ang = math.atan2(q.y - end[1], q.x - end[0])
                off = abs((ang - heading + math.pi) % (2 * math.pi) - math.pi)
                if off <= SNAP_ANGLE and (best is None or d < best[0]):
                    best = (d, q)
            if best is not None:
                out.append(LineString([end, (best[1].x, best[1].y)]))
    return out


_LEVEL = re.compile(r"^(AQ|MBC|WMC)$")


def level_labels(words, sim: Similarity, page_rect) -> list[tuple[str, str, Point]]:
    """("AQ", "3000", point) for every building-level label on the map."""
    out = []
    max_y = page_rect.height * 0.82
    for w in words:
        if not _LEVEL.match(w[1]) or w[0].y1 > max_y:
            continue
        nxt = [x for x in words if x[2] == w[2] and x[3] == w[3] and x[4] == w[4] + 1]
        if not nxt or not re.fullmatch(r"P?\d+", nxt[0][1]):
            continue
        r = w[0] | nxt[0][0]
        x, y = sim.pdf_to_map((r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2)
        out.append((w[1], nxt[0][1], Point(float(x), float(y))))
    return out


def floor_for(floors, building: str, level: str) -> str | None:
    name = f"{level} Level"
    for fid, fl in floors.items():
        if fl.building == building and fl.name == name:
            return fid
    return None


def entrances(lines, footprints, labels, floors) -> list[dict]:
    """Where a walkway meets a mapped building, and which floor it enters on."""
    out = []
    for code in MAPPED:
        fp = footprints[code]
        boundary = fp.boundary
        for line in lines:
            hit = line.intersection(boundary)
            pts = [g for g in getattr(hit, "geoms", [hit]) if isinstance(g, Point)]
            for p in pts:
                near = sorted((lab.distance(p), b, lv) for b, lv, lab in labels if b == code)
                radius = LABEL_RADIUS if near and near[0][0] <= LABEL_RADIUS else LABEL_FALLBACK
                for d, b, lv in near:
                    if d > radius:
                        break
                    fid = floor_for(floors, b, lv)
                    # The label must name a floor with corridors near this door.
                    if fid is not None and floors[fid].free_space.distance(p) <= ENTRANCE_REACH:
                        out.append({"building": b, "level": lv, "floor": fid, "point": p})
                        break
    # One entrance per 10 m of boundary and floor.
    kept = []
    for e in out:
        if all(e["floor"] != k["floor"] or e["point"].distance(k["point"]) > 10 for k in kept):
            kept.append(e)
    return kept


def outdoor_lines(lines, footprints) -> list[LineString]:
    """Walkway lines outside the mapped buildings (their insides are the indoor graph)."""
    inside = unary_union([footprints[c] for c in MAPPED])
    out = []
    for line in lines:
        rest = line.difference(inside)
        for g in getattr(rest, "geoms", [rest]):
            if isinstance(g, LineString) and g.length >= 2.0:
                out.append(g)
    return out


def extract(pdf: Path = PDF) -> dict:
    img, words, rect = _render(pdf)
    footprints = load_building_footprints()
    floors = load_floors()
    sim, overlap = georeference(img, words, rect, footprints)
    mask = _walkway_mask(img, words, rect)
    spec = GridSpec.around((-900, -500, 700, 500), GRID, 0)
    lines = trace_network(_to_map_grid(mask, sim, spec), spec)
    labels = level_labels(words, sim, rect)
    ents = entrances(lines, footprints, labels, floors)
    outdoor = outdoor_lines(lines, footprints)
    unmapped = unary_union([g for c, g in footprints.items() if c not in MAPPED])

    def ll(geom):
        def tx(x, y, z=None):
            lon, lat = xy_to_lonlat(x, y)
            return (np.round(lon, 7), np.round(lat, 7))
        from shapely.ops import transform
        return mapping(transform(tx, geom))

    feats = []
    for i, g in enumerate(outdoor):
        feats.append({"type": "Feature", "geometry": ll(g), "properties": {
            "kind": "walkway", "id": f"w{i}", "step_free": True, "verified": False,
            "through_building": bool(g.intersection(unmapped).length > 0.5 * g.length)}})
    for i, e in enumerate(ents):
        feats.append({"type": "Feature", "geometry": ll(e["point"]), "properties": {
            "kind": "entrance", "id": f"{e['building'].lower()}_{e['level'].lower()}_{i}",
            "building": e["building"], "floor": e["floor"], "level_label": f"{e['building']} {e['level']}", "verified": False}})
    return {
        "type": "FeatureCollection",
        "properties": {
            "source": str(pdf.relative_to(REPO_ROOT)),
            "method": "sfu_map.walkways: navy walkway pixels, georeferenced to campus-geometry.json footprints",
            "georeference": {"scale_m_per_pt": sim.scale, "yaw_rad": sim.yaw, "tx": sim.tx, "ty": sim.ty, "building_iou": round(overlap, 3)},
        },
        "features": feats,
    }


def write(path: Path = OUT, pdf: Path = PDF) -> dict:
    doc = extract(pdf)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=1))
    return doc
