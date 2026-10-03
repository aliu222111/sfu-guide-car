import pytest

from sfu_map.route import RouteError, RouteGraph, normalise_room


def _graph():
    n = lambda i, f, x, t="corridor": {"id": i, "type": t, "floor": f, "x": x, "y": 0.0}
    e = lambda a, b, L, t="corridor", car=True, ver=True: {"a": a, "b": b, "type": t, "length": L, "car_traversable": car, "verified": ver}
    nodes = [n("a", "F1", 0), n("b", "F1", 10), n("c", "F1", 20, "door"), n("d", "F2", 0), n("e", "F2", 10, "door")]
    edges = [e("a", "b", 10), e("b", "c", 10), e("a", "d", 8, "stairs", car=False), e("b", "d", 30, "elevator", ver=False), e("d", "e", 10)]
    dest = {"X 100": {"node": "c", "searchable": True}, "X 200": {"node": "e", "searchable": True, "name": "Lab"}}
    return RouteGraph({"nodes": nodes, "edges": edges}, dest)


def test_normalise_room():
    assert normalise_room("aq3150") == "AQ 3150"
    assert normalise_room("AQ-3150") == "AQ 3150"
    assert normalise_room(" aq 3004b ") == "AQ 3004B"
    assert normalise_room("wmcs1300") == "WMC S1300"
    assert normalise_room("AQ-5003-9") == "AQ 5003-9"


def test_resolve_and_search():
    rg = _graph()
    assert rg.resolve("x100") == "c"
    assert rg.resolve("a") == "a"
    assert rg.search("X 2") == ["X 200"]
    assert rg.search("lab") == ["X 200"]
    with pytest.raises(RouteError):
        rg.resolve("Y 1")


def test_car_avoids_stairs_walker_takes_them():
    rg = _graph()
    path, L = rg.shortest_path("a", "e")
    assert path == ["a", "b", "d", "e"] and L == 50
    path, L = rg.shortest_path("a", "e", car=False)
    assert path == ["a", "d", "e"] and L == 18
    with pytest.raises(RouteError):
        rg.shortest_path("a", "e", verified_only=True)


def test_legs():
    rg = _graph()
    path, _ = rg.shortest_path("c", "e")
    legs = rg.legs(path)
    assert [(l.kind, l.floor) for l in legs] == [("drive", "F1"), ("elevator", "F2"), ("drive", "F2")]
    assert legs[0].nodes == ["c", "b"]


def test_nearest_node():
    rg = _graph()
    assert rg.nearest_node("F2", 9, 1) == "e"
    assert rg.nearest_node("F1", 12, 0, types={"door"}) == "c"
