"""Build the campus route graph and destination index (spec section 5, outputs 2 and 3).

Per floor: corridor centreline nodes, then door, stairs, elevator and entrance
nodes attached to the nearest centreline edge that can be reached in a straight
line through free space. Floors are joined by elevator and stairs edges and
buildings by outdoor edges.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np
from shapely import STRtree
from shapely.geometry import LineString, Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import linemerge, nearest_points, unary_union

from .connectors import Connectors
from .geo import lonlat_to_xy
from .gisdata import Floor, Room
from .skeleton import centrelines

# A room whose wall is within this distance of free space gets a direct door.
DOOR_DIRECT = 1.5
# Rooms further than this from free space get no door node.
DOOR_MAX = 12.0
# Door, elevator and entrance nodes are placed this far inside the corridor,
# the first that fits: clear of the car's half-width plus Nav2's inflation.
DOOR_INSETS = (0.8, 0.6, 0.4)
# Free-space pieces closer than this are treated as joined by a doorway.
DOORWAY_GAP = 0.45
# Attach to an edge endpoint instead of splitting when this close to it.
SPLIT_SNAP = 0.5
# Stairs polygons on consecutive levels overlapping by this IoU form one stair.
STAIRS_IOU = 0.3
# Elevator edge cost: boarding and waiting, plus travel per level, metres-equivalent.
ELEVATOR_BASE = 20.0
ELEVATOR_PER_LEVEL = 4.0
STAIRS_PER_LEVEL = 8.0


@dataclass
class Graph:
    nodes: dict[str, dict] = field(default_factory=dict)
    edges: dict[tuple[str, str], dict] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)

    def add_node(self, nid: str, type: str, floor: str | None, x: float, y: float, **attrs):
        if nid in self.nodes:
            raise ValueError(f"duplicate node id {nid}")
        self.nodes[nid] = {"id": nid, "type": type, "floor": floor, "x": round(float(x), 3), "y": round(float(y), 3), **attrs}

    def add_edge(self, a: str, b: str, type: str, length: float | None = None, car_traversable: bool = True, verified: bool = True, **attrs):
        if a == b:
            return
        if length is None:
            na, nb = self.nodes[a], self.nodes[b]
            length = math.hypot(na["x"] - nb["x"], na["y"] - nb["y"])
        key = (a, b) if a < b else (b, a)
        self.edges[key] = {
            "a": key[0],
            "b": key[1],
            "type": type,
            "length": round(float(length), 3),
            "car_traversable": car_traversable,
            "verified": verified,
            **attrs,
        }

    def remove_edge(self, a: str, b: str):
        self.edges.pop((a, b) if a < b else (b, a), None)


class FloorGraphBuilder:
    """Centrelines for one floor plus attachment of point features to them."""

    def __init__(self, graph: Graph, floor: Floor, spacing: float):
        self.g = graph
        self.floor = floor
        self.free = floor.free_space
        self.free_tol = self.free.buffer(0.05)
        self._inner_cache: dict[float, BaseGeometry] = {}
        cl = centrelines(self.free, spacing=spacing)
        self.corridor_ids = []
        for i, ((x, y), j) in enumerate(zip(cl.nodes, cl.junction)):
            nid = f"{floor.id}:c{i}"
            graph.add_node(nid, "corridor_junction" if j else "corridor", floor.id, x, y)
            self.corridor_ids.append(nid)
        self.base_edges = [(self.corridor_ids[a], self.corridor_ids[b]) for a, b in cl.edges]
        for a, b in self.base_edges:
            graph.add_edge(a, b, "corridor")
        self._lines = [self._line(a, b) for a, b in self.base_edges]
        self._tree = STRtree(self._lines) if self._lines else None
        # Pending splits: base edge index -> list of (distance along, node id).
        self._splits: dict[int, list[tuple[float, str]]] = {}
        self._split_count = 0

    def _xy(self, nid):
        n = self.g.nodes[nid]
        return n["x"], n["y"]

    def _line(self, a, b):
        return LineString([self._xy(a), self._xy(b)])

    def inset_point(self, target: BaseGeometry, toward: BaseGeometry | None = None) -> Point | None:
        """A point in free space near ``target``, as far from the walls as ``DOOR_INSETS`` allow.

        Nodes are Nav2 goals, so they must sit outside the car's footprint plus
        inflation: 0.8 m from the wall where the corridor is wide enough, less
        in narrow corridors. ``toward`` picks which part of ``target`` to start
        from; by default its representative point.
        """
        if self.free.is_empty:
            return None
        src = toward if toward is not None else target.representative_point()
        _, p = nearest_points(src, self.free)
        for d in DOOR_INSETS:
            inner = self._inner(d)
            if inner.is_empty:
                continue
            _, q = nearest_points(p, inner)
            if q.distance(p) < d * 2:
                return q
        return p

    def _inner(self, d: float) -> BaseGeometry:
        if d not in self._inner_cache:
            self._inner_cache[d] = self.free.buffer(-d)
        return self._inner_cache[d]

    def attach(self, nid: str, max_candidates: int = 12) -> bool:
        """Connect an existing node to the nearest centreline edge it can see."""
        if self._tree is None:
            return False
        p = Point(self._xy(nid))
        idxs = self._tree.query_nearest(p, all_matches=True)
        cand = sorted(
            set(int(i) for i in np.atleast_1d(idxs))
            | set(int(i) for i in self._tree.query(p.buffer(30.0))),
            key=lambda i: self._lines[i].distance(p),
        )[:max_candidates]
        for i in cand:
            line = self._lines[i]
            t = line.project(p)
            q = line.interpolate(t)
            if q.distance(p) < 1e-6 or LineString([p, q]).within(self.free_tol):
                self._splits.setdefault(i, []).append((t, nid))
                return True
        return False

    def finish(self):
        """Split base edges at attachment points and connect attached nodes."""
        for i, items in self._splits.items():
            a, b = self.base_edges[i]
            line = self._lines[i]
            L = line.length
            chain = [(0.0, a)]
            for t, nid in sorted(items):
                if t < SPLIT_SNAP:
                    self.g.add_edge(nid, a, "corridor")
                    continue
                if t > L - SPLIT_SNAP:
                    self.g.add_edge(nid, b, "corridor")
                    continue
                if t - chain[-1][0] < SPLIT_SNAP and chain[-1][1] != a:
                    self.g.add_edge(nid, chain[-1][1], "corridor")
                    continue
                q = line.interpolate(t)
                sid = f"{self.floor.id}:s{self._split_count}"
                self._split_count += 1
                self.g.add_node(sid, "corridor", self.floor.id, q.x, q.y)
                chain.append((t, sid))
                self.g.add_edge(nid, sid, "corridor")
            chain.append((L, b))
            if len(chain) > 2:
                self.g.remove_edge(a, b)
                for (_, u), (_, v) in itertools.pairwise(chain):
                    self.g.add_edge(u, v, "corridor")
        self._splits.clear()


def _door_point(fb: FloorGraphBuilder, room: Room):
    """Where the room's door is assumed: middle of the wall it shares with free space."""
    d = room.geom.distance(fb.free)
    if d > DOOR_MAX:
        return None, d
    if d <= DOOR_DIRECT:
        near = room.geom.boundary.intersection(fb.free.buffer(d + 0.3))
        pieces = [g for g in getattr(near, "geoms", [near]) if g.length > 0]
        if pieces:
            merged = linemerge(pieces) if len(pieces) > 1 else pieces[0]
            longest = max(getattr(merged, "geoms", [merged]), key=lambda g: g.length)
            mid = longest.interpolate(0.5, normalized=True)
            return fb.inset_point(room.geom, toward=mid), d
    return fb.inset_point(room.geom), d


def build_graph(floors: dict[str, Floor], connectors: Connectors, spacing: float = 5.0) -> tuple[Graph, dict]:
    g = Graph()
    builders: dict[str, FloorGraphBuilder] = {}
    for fid, fl in floors.items():
        builders[fid] = FloorGraphBuilder(g, fl, spacing)

    destinations: dict[str, dict] = {}
    overrides = connectors.door_overrides()

    for fid, fb in builders.items():
        fl = fb.floor
        # Doorways between free-space pieces that GIS separates by a wall line.
        _bridge_gaps(g, fb)
        # Doors.
        for room in fl.rooms:
            if not room.gets_door:
                continue
            key = f"{room.building} {room.rm_id}"
            if key in overrides:
                ov = overrides[key]
                p = Point(ov["x"], ov["y"])
                direct = True
                dist = 0.0
            else:
                p, dist = _door_point(fb, room)
                direct = dist <= DOOR_DIRECT
            if p is None:
                g.warnings.append(f"{key}: no free space within {DOOR_MAX} m, no door node")
                continue
            nid = f"{fid}:door:{room.rm_id}"
            g.add_node(nid, "door", fid, p.x, p.y, room=room.rm_id, building=room.building, direct=direct)
            if not fb.attach(nid):
                g.warnings.append(f"{key}: door node could not see a corridor edge")
                del g.nodes[nid]
                continue
            destinations[key] = {
                "node": nid,
                "building": room.building,
                "floor": fid,
                "floor_name": fl.name,
                "room": room.rm_id,
                "type": room.rm_type,
                "category": room.rm_cat,
                "name": room.rm_name,
                "searchable": room.searchable,
                "direct_door": direct,
                "room_x": round(room.geom.centroid.x, 2),
                "room_y": round(room.geom.centroid.y, 2),
            }

    _add_stairs(g, floors, builders)
    _add_elevators(g, floors, builders, connectors)
    _add_entrances(g, floors, builders, connectors)
    _add_links(g, floors, builders, connectors)
    for fb in builders.values():
        fb.finish()
        _snap_inside(g, fb)
    _add_outdoor(g, connectors)
    return g, dict(sorted(destinations.items()))


def _snap_inside(g: Graph, fb: FloorGraphBuilder, inset: float = 0.15, max_move: float = 0.5):
    """Move nodes that rasterisation left on or just past a wall back into free space."""
    inner = fb.free.buffer(-inset)
    if inner.is_empty:
        return
    moved = set()
    for nid, n in g.nodes.items():
        if n["floor"] != fb.floor.id:
            continue
        p = Point(n["x"], n["y"])
        if fb.free.contains(p):
            continue
        q = nearest_points(p, inner)[1]
        if q.distance(p) <= max_move:
            n["x"], n["y"] = round(q.x, 3), round(q.y, 3)
            moved.add(nid)
    for e in g.edges.values():
        if e["type"] in ("corridor", "door") and (e["a"] in moved or e["b"] in moved):
            a, b = g.nodes[e["a"]], g.nodes[e["b"]]
            e["length"] = round(math.hypot(a["x"] - b["x"], a["y"] - b["y"]), 3)


def _bridge_gaps(g: Graph, fb: FloorGraphBuilder):
    parts = [p for p in getattr(fb.free, "geoms", [fb.free]) if p.area > 1.0]
    k = 0
    for a, b in itertools.combinations(parts, 2):
        d = a.distance(b)
        if d > DOORWAY_GAP:
            continue
        shared = b.boundary.intersection(a.buffer(d + 0.05))
        pieces = [s for s in getattr(shared, "geoms", [shared]) if s.length > 0]
        if pieces:
            longest = max(pieces, key=lambda s: s.length)
            mid_b = longest.interpolate(0.5, normalized=True)
        else:
            mid_b = nearest_points(a, b)[1]
        mid_a = nearest_points(mid_b, a)[1]
        pa = fb.inset_point(a, toward=mid_a)
        pb = fb.inset_point(b, toward=mid_b)
        na, nb = f"{fb.floor.id}:gap{k}a", f"{fb.floor.id}:gap{k}b"
        g.add_node(na, "doorway", fb.floor.id, pa.x, pa.y)
        g.add_node(nb, "doorway", fb.floor.id, pb.x, pb.y)
        if fb.attach(na) and fb.attach(nb):
            g.add_edge(na, nb, "door", verified=False, inferred=True)
            k += 1
        else:
            for n in (na, nb):
                g.nodes.pop(n, None)
            for key in [e for e in g.edges if na in e or nb in e]:
                del g.edges[key]
            # A pending split may reference the removed node; drop it.
            for lst in fb._splits.values():
                lst[:] = [(t, n) for t, n in lst if n not in (na, nb)]


def _add_stairs(g: Graph, floors, builders):
    by_building: dict[str, list[Floor]] = {}
    for fl in floors.values():
        by_building.setdefault(fl.building, []).append(fl)
    for bl, fls in by_building.items():
        fls.sort(key=lambda f: f.ref_lvl)
        placed: dict[tuple[str, str], str] = {}
        for fl in fls:
            fb = builders[fl.id]
            for r in fl.rooms:
                if r.rm_type != "Stairs":
                    continue
                if r.geom.distance(fb.free) > DOOR_DIRECT:
                    continue
                p = fb.inset_point(r.geom)
                nid = f"{fl.id}:stairs:{r.rm_id}"
                g.add_node(nid, "stairs", fl.id, p.x, p.y, room=r.rm_id)
                if fb.attach(nid):
                    placed[(fl.id, r.rm_id)] = nid
                else:
                    del g.nodes[nid]
        for lo, hi in itertools.pairwise(fls):
            lo_st = [r for r in lo.rooms if r.rm_type == "Stairs" and (lo.id, r.rm_id) in placed]
            hi_st = [r for r in hi.rooms if r.rm_type == "Stairs" and (hi.id, r.rm_id) in placed]
            for ra in lo_st:
                for rb in hi_st:
                    inter = ra.geom.intersection(rb.geom).area
                    if inter and inter / ra.geom.union(rb.geom).area >= STAIRS_IOU:
                        g.add_edge(
                            placed[(lo.id, ra.rm_id)],
                            placed[(hi.id, rb.rm_id)],
                            "stairs",
                            length=STAIRS_PER_LEVEL,
                            car_traversable=False,
                        )


def _connector_point(spec: dict, floors) -> Point:
    if "xy" in spec:
        return Point(*spec["xy"])
    if "latlon" in spec:
        lat, lon = spec["latlon"]
        x, y = lonlat_to_xy(lon, lat)
        return Point(float(x), float(y))
    if "shaft" in spec:
        fid, rm = spec["shaft"].split("/")
        for r in floors[fid].rooms:
            if r.rm_id == rm:
                return r.geom.centroid
        raise KeyError(f"shaft {spec['shaft']} not found in GIS rooms")
    raise KeyError(f"connector {spec.get('id')} needs xy, latlon or shaft")


def _add_elevators(g: Graph, floors, builders, connectors: Connectors):
    for e in connectors.elevators:
        centre = _connector_point(e, floors)
        stops = []
        for fid in e["floors"]:
            if fid not in builders:
                raise KeyError(f"elevator {e['id']}: unknown floor {fid}")
            fb = builders[fid]
            p = fb.inset_point(centre.buffer(1.5), toward=centre)
            nid = f"{fid}:elev:{e['id']}"
            g.add_node(nid, "elevator", fid, p.x, p.y, elevator=e["id"], level=floors[fid].ref_lvl)
            if not fb.attach(nid):
                g.warnings.append(f"elevator {e['id']} on {fid}: no corridor edge in sight")
            stops.append((floors[fid].ref_lvl, nid))
        for (la, a), (lb, b) in itertools.combinations(stops, 2):
            g.add_edge(
                a,
                b,
                "elevator",
                length=ELEVATOR_BASE + ELEVATOR_PER_LEVEL * abs(la - lb),
                verified=bool(e.get("verified", False)),
            )


def _add_entrances(g: Graph, floors, builders, connectors: Connectors):
    for ent in connectors.entrances:
        fb = builders[ent["floor"]]
        outside = _connector_point(ent, floors)
        p = fb.inset_point(outside.buffer(0.1), toward=outside)
        nid = f"entrance:{ent['id']}"
        g.add_node(
            nid,
            "entrance",
            ent["floor"],
            p.x,
            p.y,
            outside_x=round(outside.x, 3),
            outside_y=round(outside.y, 3),
            label=ent.get("label", ent["id"]),
            verified=bool(ent.get("verified", False)),
        )
        if not fb.attach(nid):
            g.warnings.append(f"entrance {ent['id']}: no corridor edge in sight")


def _add_links(g: Graph, floors, builders, connectors: Connectors):
    """Hand-entered indoor links: doorways the GIS misses, building bridges."""
    for k, link in enumerate(connectors.links):
        ends = []
        for side in ("a", "b"):
            s = link[side]
            fb = builders[s["floor"]]
            p = _connector_point(s, floors)
            p = fb.inset_point(p.buffer(0.1), toward=p)
            nid = f"{s['floor']}:link:{link['id']}:{side}"
            g.add_node(nid, "doorway", s["floor"], p.x, p.y)
            if not fb.attach(nid):
                g.warnings.append(f"link {link['id']} end {side}: no corridor edge in sight")
            ends.append(nid)
        g.add_edge(
            ends[0],
            ends[1],
            link.get("type", "door"),
            car_traversable=bool(link.get("car_traversable", True)),
            verified=bool(link.get("verified", False)),
        )


def _add_outdoor(g: Graph, connectors: Connectors):
    """Outdoor links between entrances, as hand-drawn waypoint polylines."""
    for link in connectors.outdoor_links:
        prev = f"entrance:{link['from']}"
        if prev not in g.nodes:
            raise KeyError(f"outdoor link {link['id']}: unknown entrance {link['from']}")
        n0 = g.nodes[prev]
        # Leave the building from the outside point of the entrance.
        out0 = f"outdoor:{link['id']}:start"
        g.add_node(out0, "outdoor_junction", None, n0.get("outside_x", n0["x"]), n0.get("outside_y", n0["y"]))
        g.add_edge(prev, out0, "door", verified=bool(n0.get("verified", False)))
        prev = out0
        for k, wp in enumerate(link.get("waypoints", [])):
            p = _connector_point({"latlon": wp}, {})
            nid = f"outdoor:{link['id']}:{k}"
            g.add_node(nid, "outdoor_junction", None, p.x, p.y)
            g.add_edge(prev, nid, "outdoor", car_traversable=bool(link.get("step_free", False)), verified=bool(link.get("verified", False)))
            prev = nid
        end = f"entrance:{link['to']}"
        if end not in g.nodes:
            raise KeyError(f"outdoor link {link['id']}: unknown entrance {link['to']}")
        n1 = g.nodes[end]
        out1 = f"outdoor:{link['id']}:end"
        g.add_node(out1, "outdoor_junction", None, n1.get("outside_x", n1["x"]), n1.get("outside_y", n1["y"]))
        g.add_edge(prev, out1, "outdoor", car_traversable=bool(link.get("step_free", False)), verified=bool(link.get("verified", False)))
        g.add_edge(out1, end, "door", verified=bool(n1.get("verified", False)))


def outdoor_geometry(g: Graph) -> BaseGeometry:
    """Polylines of all outdoor edges, for the outdoor occupancy grid."""
    lines = []
    for e in g.edges.values():
        a, b = g.nodes[e["a"]], g.nodes[e["b"]]
        if a["floor"] is None and b["floor"] is None:
            lines.append(LineString([(a["x"], a["y"]), (b["x"], b["y"])]))
    return unary_union(lines) if lines else LineString()
