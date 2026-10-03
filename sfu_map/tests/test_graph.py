import math

from shapely.geometry import LineString, Point, box

from sfu_map.connectors import Connectors
from sfu_map.graph import build_graph
from sfu_map.skeleton import centrelines
from sfu_map.route import RouteGraph

from conftest import room


def _route_graph(g, dest):
    return RouteGraph({"nodes": list(g.nodes.values()), "edges": list(g.edges.values())}, dest)


def test_centrelines_t_junction(t_floor):
    cl = centrelines(t_floor.free_space, spacing=5.0)
    junctions = [n for n, j in zip(cl.nodes, cl.junction) if j]
    # Three dead ends and one junction near (20, 1.5).
    assert len(junctions) == 4
    assert any(math.dist(n, (20, 1.5)) < 1.5 for n in junctions)
    for a, b in cl.edges:
        assert math.dist(cl.nodes[a], cl.nodes[b]) <= 5.0 + 0.5
        assert LineString([cl.nodes[a], cl.nodes[b]]).within(t_floor.free_space.buffer(0.05))


def test_doors_and_destinations(t_floor):
    g, dest = build_graph({t_floor.id: t_floor}, Connectors())
    assert set(dest) == {"T 101", "T 102", "T 103", "T 104"}
    door = g.nodes[dest["T 103"]["node"]]
    # Middle of the shared wall, inset into the corridor.
    assert abs(door["x"] - 24.5) < 0.5 and 0.2 < door["y"] < 1.0
    assert t_floor.free_space.contains(Point(door["x"], door["y"]))
    # Every door is connected and reachable from every other.
    rg = _route_graph(g, dest)
    path, length = rg.shortest_path(dest["T 101"]["node"], dest["T 104"]["node"])
    assert 25 < length < 45


def test_stairs_never_car_traversable(t_floor):
    upper = type(t_floor)("T", "02", "Upper", 2)
    upper.rooms = [
        room("C3", box(0, 0, 40, 3), "CIRCULATION", "Corridor", floor="T-02"),
        room("S2", box(40.2, 0, 44, 3), "PENETRATIONS", "Stairs", floor="T-02"),
        room("201", box(0, -6.2, 9, -0.2), floor="T-02"),
    ]
    floors = {t_floor.id: t_floor, upper.id: upper}
    g, dest = build_graph(floors, Connectors())
    stairs = [e for e in g.edges.values() if e["type"] == "stairs"]
    assert len(stairs) == 1 and not stairs[0]["car_traversable"]
    rg = _route_graph(g, dest)
    rg.shortest_path(dest["T 101"]["node"], dest["T 201"]["node"], car=False)
    try:
        rg.shortest_path(dest["T 101"]["node"], dest["T 201"]["node"], car=True)
        raise AssertionError("car routed over stairs")
    except Exception as e:
        assert "no route" in str(e)

    # An elevator from connectors.yaml makes it car-routable, unverified.
    conn = Connectors(elevators=[{"id": "L1", "xy": [30, 1.5], "floors": ["T-01", "T-02"]}])
    g, dest = build_graph(floors, conn)
    rg = _route_graph(g, dest)
    path, _ = rg.shortest_path(dest["T 101"]["node"], dest["T 201"]["node"])
    legs = rg.legs(path)
    assert [l.kind for l in legs] == ["drive", "elevator", "drive"]
    try:
        rg.shortest_path(dest["T 101"]["node"], dest["T 201"]["node"], verified_only=True)
        raise AssertionError("routed over an unverified elevator")
    except Exception as e:
        assert "no route" in str(e)


def test_wall_line_gap_becomes_door(t_floor):
    # A side corridor separated from the main one by a 0.2 m wall line.
    t_floor.rooms.append(room("C9", box(0, 3.2, 6, 12), "CIRCULATION", "Corridor"))
    t_floor.rooms.append(room("110", box(6.2, 6, 12, 12)))
    g, dest = build_graph({t_floor.id: t_floor}, Connectors())
    doors = [e for e in g.edges.values() if e["type"] == "door"]
    assert len(doors) == 1 and doors[0]["verified"] is False
    rg = _route_graph(g, dest)
    path, _ = rg.shortest_path(dest["T 103"]["node"], dest["T 110"]["node"])
    assert "door" in [l.kind for l in rg.legs(path)]
