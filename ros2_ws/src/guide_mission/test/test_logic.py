import math

import pytest

from guide_mission.alignment import Similarity2D, load_alignments, save_alignment, solve
from guide_mission.follow import FollowMonitor, FollowParams
from guide_mission.instructions import Turn, arrival_text, find_turns, turn_text
from guide_mission.signs import match_sign
from sfu_map.route import RouteGraph


def test_similarity_recovers_transform_and_inverse(tmp_path):
    truth = Similarity2D(1.02, math.radians(37), 12.0, -5.0)
    src = [(0, 0), (10, 0), (10, 30), (-4, 7)]
    dst = [truth.apply(*p) for p in src]
    sim = solve(src, dst)
    assert abs(sim.scale - 1.02) < 1e-9 and abs(sim.yaw - truth.yaw) < 1e-9 and sim.rms < 1e-9
    inv = sim.inverse()
    x, y = inv.apply(*sim.apply(3.0, 4.0))
    assert abs(x - 3) < 1e-9 and abs(y - 4) < 1e-9
    save_alignment(tmp_path / "a.yaml", "AQ-03", sim, [("n", (0, 0))])
    loaded = load_alignments(tmp_path / "a.yaml")["AQ-03"]
    assert abs(loaded.tx - sim.tx) < 1e-9


def test_similarity_without_scale_and_noise():
    truth = Similarity2D(1.0, -1.0, 3.0, 4.0)
    src = [(0, 0), (20, 0), (0, 15)]
    dst = [(x + 0.1, y - 0.1) for x, y in (truth.apply(*p) for p in src)]
    sim = solve(src, dst, allow_scale=False)
    assert sim.scale == 1.0 and abs(sim.yaw + 1.0) < 0.02


def test_follow_monitor():
    f = FollowMonitor(FollowParams())
    assert not f.following(0.0)
    f.heartbeat(10.0)
    assert f.following(12.4) and not f.following(12.6)  # one missed beat is tolerated
    f.heartbeat(12.0)
    f.rssi(-90, 12.0)
    assert f.following(13.0) and not f.following(15.1)
    f.rssi(-60, 15.2)
    f.heartbeat(15.2)
    assert f.following(15.3)
    f.voice_mode = True
    assert f.following(100.0)


def _l_graph():
    nodes = [{"id": f"n{i}", "type": "corridor", "floor": "AQ-03", "x": x, "y": y} for i, (x, y) in enumerate([(0, 0), (5, 0), (10, 0), (10, 5), (10, 10)])]
    nodes.append({"id": "AQ-03:door:3150", "type": "door", "floor": "AQ-03", "x": 9.6, "y": 12.0})
    edges = [{"a": f"n{i}", "b": f"n{i+1}", "type": "corridor", "length": 5, "car_traversable": True, "verified": True} for i in range(4)]
    edges.append({"a": "n4", "b": "AQ-03:door:3150", "type": "corridor", "length": 2, "car_traversable": True, "verified": True})
    dest = {"AQ 3150": {"node": "AQ-03:door:3150", "floor": "AQ-03", "room_x": 5.0, "room_y": 12.0}}
    return RouteGraph({"nodes": nodes, "edges": edges}, dest)


def test_turns_and_text():
    rg = _l_graph()
    path = ["n0", "n1", "n2", "n3", "n4"]
    turns = find_turns(rg, path)
    assert len(turns) == 1 and turns[0].node == "n2" and turns[0].direction == "left"
    assert turn_text(turns[0], 4.6) == "In 5 metres, turn left."
    assert Turn("x", -math.radians(45)).direction == "slight right"


def test_arrival_side():
    rg = _l_graph()
    # Travelling north up x=10; the room centre is west of the door: on the left.
    assert arrival_text(rg, ["n2", "n3", "n4", "AQ-03:door:3150"], "AQ 3150") == "We've arrived. AQ 3150 is on your left."


def test_sign_matching():
    rg = _l_graph()
    fix = match_sign(rg, "AQ 3150", "AQ-03", 8.0, 9.0)
    assert fix and fix.node == "AQ-03:door:3150"
    assert match_sign(rg, "3150", "AQ-03", 8.0, 9.0).room == "AQ 3150"
    assert match_sign(rg, "3150", "AQ-03", 50.0, 50.0) is None  # too far from the estimate
    assert match_sign(rg, "3150", "AQ-03", 50.0, 50.0, lost=True) is not None
    assert match_sign(rg, "3150", "AQ-05", 8.0, 9.0) is None  # wrong floor
    assert match_sign(rg, "EXIT", "AQ-03", 8.0, 9.0) is None
    assert match_sign(rg, "3150", None, 8.0, 9.0) is None


def test_follow_rear_camera():
    f = FollowMonitor(FollowParams())
    f.voice_mode = True
    assert f.following(0.0)
    f.student(2.0, 1.0)
    assert f.following(4.0) and not f.following(5.1)  # out of view for over 4 s
    f.student(8.0, 6.0)
    assert f.following(7.0) and not f.following(9.1)  # too far for 3 s
    f.student(float("inf"), 9.2)
    f.student(2.0, 9.5)
    assert f.following(9.6)


def test_waypoints_sparse_but_keep_turns_and_goal(campus):
    from guide_mission.instructions import waypoints

    path, _ = campus.shortest_path("entrance:aq_convocation_mall", campus.resolve("AQ 3150"))
    wp = waypoints(campus, path)
    assert wp[0] == path[0] and wp[-1] == path[-1]
    assert len(wp) < len(path) / 2
    turns = {t.node for t in find_turns(campus, path)}
    assert turns <= set(wp) | {path[0]}  # every turn far enough from the last waypoint is kept
    pts = [campus.nodes[n] for n in wp]
    for a, b in zip(pts, pts[1:]):
        assert math.hypot(b["x"] - a["x"], b["y"] - a["y"]) <= 25.0
