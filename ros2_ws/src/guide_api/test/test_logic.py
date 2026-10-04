"""Phases, progress and frames: the pure pieces."""

import math

from guide_api import progress
from guide_api.frames import Affine, AppFrames, fit
from guide_api.phases import app_phase
from guide_api.service import normalise_key


def test_phases_by_stage():
    assert app_phase("summon", "driving")[0] == "enroute"
    assert app_phase("guide", "routing")[0] == "guiding"
    assert app_phase("summon", "arrived")[0] == "arrived"
    assert app_phase("guide", "arrived")[0] == "done"
    assert app_phase("guide", "aborted", "cancelled")[0] == "aborted"
    assert app_phase("guide", "idle") == (None, None)


def test_help_prompts_use_the_cars_own_words():
    phase, prompt = app_phase("guide", "waiting_for_help", "door")
    assert phase == "help" and prompt.button == "I'm holding the door"
    phase, prompt = app_phase("guide", "waiting_for_student", "still_with_me")
    assert phase == "waiting" and prompt.button == "Yes, I'm here"
    phase, prompt = app_phase("guide", "waiting_for_help", "something_new")
    assert phase == "help" and prompt.button  # an unknown request still gets a button


ROUTE = [
    {"id": "a", "x": 0.0, "y": 0.0, "floor": "ASB-09"},
    {"id": "b", "x": 10.0, "y": 0.0, "floor": "ASB-09"},
    {"id": "e9", "x": 10.0, "y": 5.0, "floor": "ASB-09"},
    {"id": "e10", "x": 10.0, "y": 5.0, "floor": "ASB-10"},
    {"id": "d", "x": 10.0, "y": 25.0, "floor": "ASB-10"},
]


def test_route_length_skips_the_elevator():
    assert progress.route_length(ROUTE) == 35.0
    assert progress.floor_changes(ROUTE) == 1


def test_remaining_from_a_pose_beside_the_route():
    left, rides = progress.remaining(ROUTE, (4.0, 0.6, 0.0), "ASB-09")
    assert math.isclose(left, 31.0) and rides == 1
    left, rides = progress.remaining(ROUTE, (10.2, 15.0, 0.0), "ASB-10")
    assert math.isclose(left, 10.0) and rides == 0


def test_estimate_counts_the_ride_and_clamps():
    est = progress.estimate(ROUTE, (0.0, 0.0, 0.0), "ASB-09", speed=0.5, ride_s=60.0)
    assert est == {"progress": 0.0, "leftM": 35.0, "etaSec": 130.0}
    assert progress.estimate(ROUTE, None, "ASB-09", 0.5) is None
    assert progress.estimate(ROUTE, (0.0, 0.0, 0.0), "AQ-03", 0.5) is None  # not on any stretch of this route


def test_affine_fit_and_inverse():
    truth = Affine(0.0858, -0.0198, 119.3, -0.02, -0.086, -96.6)
    pts = [(100.0, 200.0), (900.0, 150.0), (500.0, 1000.0), (300.0, 640.0)]
    got = fit([(p, truth.apply(*p)) for p in pts])
    for p in pts:
        assert math.dist(got.apply(*p), truth.apply(*p)) < 1e-6
        assert math.dist(got.inverse().apply(*got.apply(*p)), p) < 1e-6


def test_app_frames_point_and_floor():
    frames = AppFrames({"ASB": Affine(0.1, 0.0, 50.0, 0.0, -0.1, 20.0)})
    assert AppFrames.split_floor("ASB-09") == ("ASB", "9")
    assert AppFrames.split_floor("ASB-10") == ("ASB", "10")
    assert AppFrames.split_floor(None) is None
    assert frames.point("ASB-09", 60.0, 10.0) == {"lv": "9", "x": 100.0, "y": 100.0}
    assert frames.point("AQ-03", 0.0, 0.0) is None  # no transform for that building


def test_normalise_key():
    assert normalise_key("asb9896") == "ASB 9896"
    assert normalise_key(" ASB-9896 ") == "ASB 9896"
    assert normalise_key("aq 3046") == "AQ 3046"
    assert normalise_key("") == ""
