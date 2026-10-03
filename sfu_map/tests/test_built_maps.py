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
    assert 4000 < free.sum() * spec.resolution**2 < 5000  # spec: 4,412 m2 of corridor


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
