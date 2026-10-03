"""Phase 0 exit criteria (spec section 11) against the committed maps/ output."""

from pathlib import Path

import pytest

from sfu_map.raster import load_nav2_map
from sfu_map.route import RouteGraph

MAPS = Path(__file__).resolve().parents[2] / "maps"
pytestmark = pytest.mark.skipif(not (MAPS / "route_graph.json").exists(), reason="run `sfu-map build` first")


@pytest.fixture(scope="module")
def rg():
    return RouteGraph.load(MAPS / "route_graph.json")


def test_convocation_mall_to_aq_3150(rg):
    path, length = rg.shortest_path(rg.resolve("aq_convocation_mall"), rg.resolve("AQ 3150"))
    assert path[-1] == "AQ-03:door:3150"
    assert {rg.nodes[n]["floor"] for n in path} == {"AQ-03"}
    assert 50 < length < 400


def test_every_classroom_is_a_destination(rg):
    classrooms = [k for k, d in rg.destinations.items() if d["category"] == "CLASSROOM"]
    assert len(classrooms) >= 90
    for b in ("AQ", "WMC", "MBC"):
        assert any(k.startswith(b + " ") for k in classrooms)


def test_aq_3000_grid_loads_and_door_is_free(rg):
    free, spec = load_nav2_map(MAPS / "floors" / "AQ-03.yaml")
    assert spec.resolution == 0.05
    door = rg.nodes["AQ-03:door:3150"]
    c, r = spec.to_pixel(door["x"], door["y"])
    assert free[int(r), int(c)]
    # 4,412 m2 of corridor (spec section 5) plus public lounges and study areas and doorway openings.
    assert 4400 < free.sum() * spec.resolution**2 < 6500


def test_floor_nodes_lie_in_free_space(rg):
    for fid in ("AQ-03", "MBC-01", "WMC-02"):
        free, spec = load_nav2_map(MAPS / "floors" / f"{fid}.yaml")
        on_floor = [n for n in rg.nodes.values() if n["floor"] == fid]
        bad = []
        for n in on_floor:
            c, r = spec.to_pixel(n["x"], n["y"])
            if not free[int(r), int(c)]:
                bad.append(n["id"])
        # Merged junctions in narrow spots can sit a pixel into a wall.
        assert len(bad) <= 0.01 * len(on_floor), bad[:10]


def test_graph_agrees_with_nav2_grids(rg):
    """Every set of nodes the graph connects is drivable on the grid for the car's width."""
    from sfu_map.checks import grid_splits

    class G:  # grid_splits wants graph.nodes / graph.edges like the builder's Graph
        nodes = rg.nodes
        edges = {(e["a"], e["b"]): e for lst in rg.adj.values() for _, e in lst}

    for fid in ("AQ-03", "AQ-05", "MBC-01", "WMC-01", "WMC-02"):
        free, spec = load_nav2_map(MAPS / "floors" / f"{fid}.yaml")
        assert grid_splits(G, fid, free, spec) == []


def test_doorways_are_open_in_the_grids(rg):
    """The grid has a gap wherever the graph has a same-floor door edge."""
    from scipy import ndimage

    by_floor = {}
    for lst in rg.adj.values():
        for _, e in lst:
            a, b = rg.nodes[e["a"]], rg.nodes[e["b"]]
            if e["type"] == "door" and a["floor"] and a["floor"] == b["floor"]:
                by_floor.setdefault(a["floor"], set()).add((e["a"], e["b"]))
    assert by_floor
    for fid, edges in by_floor.items():
        free, spec = load_nav2_map(MAPS / "floors" / f"{fid}.yaml")
        regions, _ = ndimage.label(ndimage.distance_transform_edt(free) * spec.resolution > 0.19)
        for a, b in edges:
            la, lb = (regions[int(r), int(c)] for c, r in (spec.to_pixel(rg.nodes[n]["x"], rg.nodes[n]["y"]) for n in (a, b)))
            assert la and la == lb, (a, b)


def test_buildings_connected_by_accessible_walkways(rg):
    """AQ, MBC and WMC are joined by the step-free walkways from the 2024 accessibility map."""
    for a, b in (("AQ 3150", "MBC 2290"), ("AQ 3150", "WMC 2260"), ("MBC 2290", "WMC 3510")):
        path, _ = rg.shortest_path(rg.resolve(a), rg.resolve(b))
        assert any(rg.nodes[n]["floor"] is None for n in path), (a, b)  # goes outside


def test_most_destinations_reachable(rg):
    from collections import deque

    seen, q = {"entrance:aq_convocation_mall"}, deque(["entrance:aq_convocation_mall"])
    while q:
        for v, e in rg.adj[q.popleft()]:
            if e["car_traversable"] and v not in seen:
                seen.add(v)
                q.append(v)
    ok = sum(d["node"] in seen for d in rg.destinations.values())
    assert ok >= 0.9 * len(rg.destinations)
