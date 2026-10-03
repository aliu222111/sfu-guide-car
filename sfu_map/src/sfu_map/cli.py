"""Command line: ``sfu-map build``, ``sfu-map route``, ``sfu-map candidates``."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

DEFAULT_OUT = Path(__file__).resolve().parents[3] / "maps"


def cmd_build(args) -> int:

    from . import geo
    from .connectors import load_connectors
    from .gisdata import load_floors
    from .graph import build_graph, outdoor_geometry
    from .preview import render_floor
    from .raster import GridSpec, rasterize, write_nav2_map
    from .route import RouteError, RouteGraph

    out = Path(args.out)
    floors = load_floors(args.rooms)
    connectors = load_connectors(args.connectors)
    ext = "." + args.format

    # One grid extent per building so every floor of it shares an origin.
    building_bounds = {}
    for fl in floors.values():
        b = fl.footprint.bounds
        old = building_bounds.get(fl.building, b)
        building_bounds[fl.building] = (min(old[0], b[0]), min(old[1], b[1]), max(old[2], b[2]), max(old[3], b[3]))

    walkways = None
    if args.walkways and Path(args.walkways).exists():
        walkways = json.loads(Path(args.walkways).read_text())
    graph, destinations = build_graph(floors, connectors, spacing=args.spacing, walkways=walkways)
    openings = doorway_openings(graph)

    floor_meta = {}
    specs = {}
    masks = {}
    for fid, fl in floors.items():
        spec = GridSpec.around(building_bounds[fl.building], args.resolution, margin=2.0)
        specs[fid] = spec
        mask = rasterize(fl.free_space, spec)
        masks[fid] = mask.copy()  # GIS free space only, for the graph check
        if fid in openings:
            # Doorways the graph crosses: the GIS draws a wall line there.
            mask |= rasterize(openings[fid], spec)
        yaml_path = write_nav2_map(mask, spec, out / "floors" / f"{fid}{ext}")
        floor_meta[fid] = {
            "building": fl.building,
            "fl_id": fl.fl_id,
            "name": fl.name,
            "level": fl.ref_lvl,
            "grid": str(yaml_path.relative_to(out)),
            "free_area_m2": round(fl.free_space.area, 1),
        }
        print(f"grid {fid:8s} {spec.width}x{spec.height} free {fl.free_space.area:7.0f} m2", file=sys.stderr)


    outdoor = outdoor_geometry(graph)
    if not outdoor.is_empty:
        area = outdoor.buffer(args.outdoor_half_width)
        spec = GridSpec.around(area.bounds, args.outdoor_resolution, margin=2.0)
        yaml_path = write_nav2_map(rasterize(area, spec), spec, out / f"outdoor{ext}")
        floor_meta["outdoor"] = {"building": None, "name": "Outdoor paths", "level": None, "grid": str(yaml_path.relative_to(out))}

    doc = {
        "format": "sfu_map route graph v1",
        "frame": geo.frame_info(),
        "floors": floor_meta,
        "nodes": sorted(graph.nodes.values(), key=lambda n: n["id"]),
        "edges": sorted(graph.edges.values(), key=lambda e: (e["a"], e["b"])),
    }
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "route_graph.json", "w") as f:
        json.dump(doc, f, separators=(",", ":"))
    with open(out / "destinations.json", "w") as f:
        json.dump(destinations, f, indent=1)

    report = _report(floors, graph, destinations, connectors)
    from .checks import grid_splits

    splits = [m for fid in floors for m in grid_splits(graph, fid, masks[fid], specs[fid])]
    report.append(f"graph pieces the Nav2 grid splits for the car ({len(splits)}; must be 0):")
    report.extend("  " + m for m in splits)
    rg = RouteGraph(doc, destinations)
    for start, goal in args.check:
        try:
            path, length = rg.shortest_path(rg.resolve(start), rg.resolve(goal))
            report.append(f"check {start} -> {goal}: {length:.0f} m, {len(path)} nodes")
        except RouteError as e:
            report.append(f"check {start} -> {goal}: FAILED, {e}")
    (out / "build_report.txt").write_text("\n".join(report) + "\n")
    print("\n".join(report[:40]), file=sys.stderr)

    if args.previews:
        for fid, fl in floors.items():
            spec = GridSpec.around(building_bounds[fl.building], args.preview_resolution, margin=2.0)
            render_floor(fl, graph, spec, out / "previews" / f"{fid}.png")
    return 0


def _components(graph, floor: str) -> list[int]:
    """Sizes of connected pieces of one floor's graph using only same-floor edges."""
    parent = {nid: nid for nid, n in graph.nodes.items() if n["floor"] == floor}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for e in graph.edges.values():
        if e["a"] in parent and e["b"] in parent:
            parent[find(e["a"])] = find(e["b"])
    return sorted(Counter(find(n) for n in parent).values(), reverse=True)


def _reachability(graph, destinations, start: str = "entrance:aq_convocation_mall") -> list[str]:
    """Which destinations the car can reach from ``start`` (car-traversable edges only)."""
    from collections import deque

    if start not in graph.nodes:
        return [f"reachability: no start node {start}"]
    adj: dict[str, list[str]] = {}
    for e in graph.edges.values():
        if e["car_traversable"]:
            adj.setdefault(e["a"], []).append(e["b"])
            adj.setdefault(e["b"], []).append(e["a"])
    seen, q = {start}, deque([start])
    while q:
        for v in adj.get(q.popleft(), []):
            if v not in seen:
                seen.add(v)
                q.append(v)
    by_floor, ok = Counter(), Counter()
    for d in destinations.values():
        by_floor[d["floor"]] += 1
        ok[d["floor"]] += d["node"] in seen
    cls = sorted(k for k, d in destinations.items() if d["category"] == "CLASSROOM")
    lost = [k for k in cls if destinations[k]["node"] not in seen]
    out = [f"car-reachable from {start}: {sum(ok.values())} of {len(destinations)} destinations, {len(cls) - len(lost)} of {len(cls)} classrooms"]
    out.append("  " + ", ".join(f"{f} {ok[f]}/{by_floor[f]}" for f in sorted(by_floor)))
    if lost:
        out.append("  classrooms not reachable: " + ", ".join(lost))
    return out


def _report(floors, graph, destinations, connectors) -> list[str]:
    lines = ["sfu_map build report", ""]
    lines.append(f"nodes {len(graph.nodes)}, edges {len(graph.edges)}, destinations {len(destinations)}")
    lines.append("node types: " + ", ".join(f"{k} {v}" for k, v in sorted(Counter(n["type"] for n in graph.nodes.values()).items())))
    lines.append("edge types: " + ", ".join(f"{k} {v}" for k, v in sorted(Counter(e["type"] for e in graph.edges.values()).items())))
    unverified = sum(1 for e in graph.edges.values() if not e["verified"])
    lines.append(f"edges not yet verified on site: {unverified}")
    lines.append(
        f"connectors: {len(connectors.elevators)} elevators, {len(connectors.entrances)} entrances, "
        f"{len(connectors.links)} links, {len(connectors.outdoor_links)} outdoor links, {len(connectors.doors)} door overrides"
    )
    lines.append("")
    classrooms = [r for fl in floors.values() for r in fl.rooms if r.rm_cat == "CLASSROOM" and r.gets_door]
    missing = [f"{r.building} {r.rm_id}" for r in classrooms if f"{r.building} {r.rm_id}" not in destinations]
    lines.append(f"classroom-category rooms with a door node: {len(classrooms) - len(missing)} of {len(classrooms)}")
    if missing:
        lines.append("  missing: " + ", ".join(sorted(missing)))
    lines.append("")
    lines.append("per floor: graph pieces (node counts; more than one piece needs a link in connectors.yaml)")
    for fid in floors:
        comps = _components(graph, fid)
        lines.append(f"  {fid:8s} {len(comps):2d} pieces {comps[:8]}")
    lines.append("")
    lines.extend(_reachability(graph, destinations))
    lines.append("")
    lines.append(f"warnings ({len(graph.warnings)}):")
    lines.extend("  " + w for w in graph.warnings)
    lines.append("")
    return lines


DOOR_WIDTH = 0.9  # metres carved into the grid along each door edge


def doorway_openings(graph) -> dict:
    """Per floor, the strips to open in the grid along same-floor door edges."""
    from shapely.geometry import LineString
    from shapely.ops import unary_union

    strips: dict[str, list] = {}
    for e in graph.edges.values():
        a, b = graph.nodes[e["a"]], graph.nodes[e["b"]]
        if e["type"] == "door" and a["floor"] and a["floor"] == b["floor"]:
            line = LineString([(a["x"], a["y"]), (b["x"], b["y"])])
            strips.setdefault(a["floor"], []).append(line.buffer(DOOR_WIDTH / 2, cap_style="flat"))
    return {f: unary_union(v) for f, v in strips.items()}


def cmd_route(args) -> int:
    from .route import RouteError, RouteGraph, describe

    rg = RouteGraph.load(Path(args.out) / "route_graph.json")
    ends = []
    for place in (args.start, args.goal):
        try:
            ends.append(rg.resolve(place))
        except RouteError as e:
            print(f"error: {e}", file=sys.stderr)
            suggestions = rg.search(place)
            if suggestions:
                print("did you mean: " + ", ".join(suggestions), file=sys.stderr)
            return 1
    try:
        path, length = rg.shortest_path(*ends, car=not args.walk, verified_only=args.verified_only)
    except RouteError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(describe(rg, path, length))
    return 0


def cmd_walkways(args) -> int:
    from . import walkways

    doc = walkways.write(Path(args.walk_out))
    feats = doc["features"]
    n_w = sum(1 for f in feats if f["properties"]["kind"] == "walkway")
    ents = [f["properties"] for f in feats if f["properties"]["kind"] == "entrance"]
    print(f"{n_w} walkway lines, {len(ents)} entrances: " + ", ".join(f"{e['id']} ({e['floor']})" for e in ents))
    print(f"georeference: {doc['properties']['georeference']}")
    return 0


def cmd_candidates(args) -> int:
    import yaml

    from .candidates import elevator_candidates
    from .gisdata import load_floors

    yaml.safe_dump({"elevators": elevator_candidates(load_floors(args.rooms))}, sys.stdout, sort_keys=False, default_flow_style=None)
    return 0


def main(argv=None) -> int:
    from .connectors import DEFAULT_CONNECTORS
    from .gisdata import DEFAULT_ROOMS, REPO_ROOT

    DEFAULT_WALKWAYS = REPO_ROOT / "data" / "sfu_derived" / "accessible_walkways.geojson"

    p = argparse.ArgumentParser(prog="sfu-map", description=__doc__)
    p.add_argument("--out", default=str(DEFAULT_OUT), help="output directory (default: repo maps/)")
    p.add_argument("--rooms", default=str(DEFAULT_ROOMS), help="GIS room polygons GeoJSON")
    sub = p.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="build floor grids, route graph and destination index")
    b.add_argument("--connectors", default=str(DEFAULT_CONNECTORS))
    b.add_argument("--walkways", default=str(DEFAULT_WALKWAYS), help="accessible walkway GeoJSON from `sfu-map walkways` ('' to skip)")
    b.add_argument("--resolution", type=float, default=0.05, help="floor grid metres per pixel")
    b.add_argument("--outdoor-resolution", type=float, default=0.10)
    b.add_argument("--outdoor-half-width", type=float, default=1.25, help="path centreline buffer, metres")
    b.add_argument("--spacing", type=float, default=5.0, help="max metres between corridor nodes")
    b.add_argument("--format", choices=["png", "pgm"], default="png")
    b.add_argument("--previews", action="store_true", help="also write previews/<floor>.png")
    b.add_argument("--preview-resolution", type=float, default=0.15)
    b.add_argument(
        "--check",
        nargs=2,
        action="append",
        metavar=("FROM", "TO"),
        default=[["aq_convocation_mall", "AQ 3150"]],
        help="route to verify and record in the build report",
    )
    b.set_defaults(func=cmd_build)

    r = sub.add_parser("route", help="print a route between two places")
    r.add_argument("start", help="room (AQ 3150), entrance id, or node id")
    r.add_argument("goal")
    r.add_argument("--walk", action="store_true", help="allow stairs (what a student on foot could take)")
    r.add_argument("--verified-only", action="store_true", help="only edges verified on site")
    r.set_defaults(func=cmd_route)

    c = sub.add_parser("candidates", help="print draft elevator entries for connectors.yaml")
    c.set_defaults(func=cmd_candidates)

    w = sub.add_parser("walkways", help="extract accessible walkways and entrances from the 2024 accessibility map PDF (needs pymupdf)")
    w.add_argument("--out", dest="walk_out", default=str(DEFAULT_WALKWAYS))
    w.set_defaults(func=cmd_walkways)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
