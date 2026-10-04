import math

from guide_mission.trip import (
    CancelNav, Creep, DriveEnable, Navigate, Recovery, SetLocalization, SetPose, Speak, StateChanged, Trip, TripParams,
)


def kinds(actions):
    return [type(a).__name__ for a in actions]


def states(actions):
    return [a.state for a in actions if isinstance(a, StateChanged)]


def new_trip(campus, **kw):
    """A trip on AQ-03 with localization set up there, as the mission node starts it."""
    trip = Trip(campus, TripParams(**kw))
    trip.floor = "AQ-03"
    return trip


def drive_to_end(trip, nav):
    """Pretend Nav2 drove the leg: report positions along it, then success."""
    out = []
    for nid in nav.nodes:
        n = trip.rg.nodes[nid]
        out += trip.on_position(n["x"], n["y"])
    return out + trip.on_nav_result(True)


def test_single_floor_trip(campus):
    trip = new_trip(campus)
    out = trip.start("AQ 3150", "entrance:aq_convocation_mall")
    assert states(out) == ["routing", "driving"]
    nav = next(a for a in out if isinstance(a, Navigate))
    assert nav.floor == "AQ-03" and nav.nodes[-1] == "AQ-03:door:3150"
    out = drive_to_end(trip, nav)
    spoken = [a.text for a in out if isinstance(a, Speak)]
    assert any("turn" in t for t in spoken), spoken
    assert trip.state == "arrived"
    assert any("AQ 3150" in t and "arrived" in t for t in spoken)
    assert any(isinstance(a, DriveEnable) and not a.on for a in out)


def test_unknown_destination_aborts(campus):
    trip = new_trip(campus)
    out = trip.start("AQ 99999", "entrance:aq_convocation_mall")
    assert trip.state == "aborted"
    assert any(isinstance(a, Speak) for a in out)


def test_elevator_protocol(campus):
    trip = new_trip(campus)
    out = trip.start("AQ 6106", "entrance:aq_convocation_mall")
    nav = next(a for a in out if isinstance(a, Navigate))
    out = drive_to_end(trip, nav)
    assert trip.state == "waiting_for_help" and "elevator" in trip.leg.kind
    assert any("call button" in a.text for a in out if isinstance(a, Speak))
    # Doors open: car boards.
    out = trip.on_confirm()
    assert any(isinstance(a, Creep) for a in out)
    assert trip.on_confirm() == []  # pressing again while boarding does nothing
    out = trip.on_creep_done()
    assert any("level" in a.text for a in out if isinstance(a, Speak))
    # Arrived: localization switches to the new floor and the car creeps out.
    out = trip.on_confirm()
    assert any(isinstance(a, SetLocalization) and a.floor == "AQ-06" for a in out)
    assert any(isinstance(a, Creep) and abs(a.turn - math.pi) < 1e-9 for a in out)  # turn round, drive out
    out = trip.on_creep_done()
    assert trip.state == "lost"
    assert any(isinstance(a, SetPose) for a in out) and any(isinstance(a, Recovery) and a.on for a in out)
    # A sign read or loop closure arrives.
    out = trip.on_localization(True)
    assert any(isinstance(a, Recovery) and not a.on for a in out)
    nav = next(a for a in out if isinstance(a, Navigate))
    assert nav.floor == "AQ-06"
    out = drive_to_end(trip, nav)
    # The rest of the route on AQ-06 crosses two inferred doorways: hold, confirm, drive.
    doors = 0
    for _ in range(10):
        if trip.state == "arrived":
            break
        if trip.state == "waiting_for_help" and trip.waiting_for == "door":
            doors += 1
            out = trip.on_confirm()
        nav = next(a for a in out if isinstance(a, Navigate))
        out = drive_to_end(trip, nav)
    assert trip.state == "arrived" and doors == 2


def test_follow_loss_holds_and_resumes_from_progress(campus):
    trip = new_trip(campus)
    out = trip.start("AQ 3150", "entrance:aq_convocation_mall")
    nav = next(a for a in out if isinstance(a, Navigate))
    mid = trip.rg.nodes[nav.nodes[10]]
    trip.on_position(mid["x"], mid["y"])
    out = trip.on_follow(False)
    assert trip.state == "waiting_for_student" and any(isinstance(a, CancelNav) for a in out)
    out = trip.on_follow(True)
    nav2 = next(a for a in out if isinstance(a, Navigate))
    assert nav2.nodes[0] == nav.nodes[10] and nav2.nodes[-1] == nav.nodes[-1]


def test_pause_wins_over_follow(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    trip.on_pause(True)
    assert trip.state == "waiting_for_student"
    assert trip.on_follow(False) == []
    assert trip.on_follow(True) == []  # still paused
    out = trip.on_pause(False)
    assert any(isinstance(a, Navigate) for a in out)


def test_localization_loss_stops_and_rotates(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    out = trip.on_localization(False)
    assert trip.state == "lost"
    assert kinds(out)[:2] == ["CancelNav", "Recovery"]
    out = trip.on_localization(True)
    assert trip.state == "driving"


def test_nav_failure_retries_then_waits(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    for _ in range(trip.params.nav_retries):
        out = trip.on_nav_result(False)  # transient: driven again without asking
        assert trip.state == "driving" and any(isinstance(a, Navigate) for a in out)
    trip.on_nav_result(False)
    assert trip.state == "waiting_for_help"
    out = trip.on_confirm()
    assert any(isinstance(a, Navigate) for a in out)


def test_voice_mode_asks_still_with_me(campus):
    trip = new_trip(campus, voice_mode=True)
    out = trip.start("AQ 3150", "entrance:aq_convocation_mall")
    nav = next(a for a in out if isinstance(a, Navigate))
    for nid in nav.nodes:
        n = trip.rg.nodes[nid]
        out = trip.on_position(n["x"], n["y"])
        if trip.state == "waiting_for_student":
            break
    assert any(a.text == "Still with me?" for a in out if isinstance(a, Speak))
    out = trip.on_confirm()
    assert trip.state == "driving" and any(isinstance(a, Navigate) for a in out)


def test_stale_nav_result_ignored(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    trip.on_pause(True)
    assert trip.on_nav_result(True) == []
    assert trip.state == "waiting_for_student"


def test_first_leg_on_another_floor_waits_for_a_fix(campus):
    trip = Trip(campus)  # localization still set up outdoors
    out = trip.start("AQ 3150", "entrance:aq_convocation_mall")
    assert any(isinstance(a, SetLocalization) and a.floor == "AQ-03" for a in out)
    assert trip.state == "lost" and not any(isinstance(a, Navigate) for a in out)
    out = trip.on_localization(True)
    assert any(isinstance(a, Navigate) for a in out)


def test_trip_asked_before_a_fix_is_replanned_from_the_fix(campus):
    trip = new_trip(campus)
    trip.localized = False  # cold start: no sign read yet, the estimate is a guess
    out = trip.start("AQ 3150", "AQ-03:door:3046")  # planned from the guess
    assert trip.state == "lost" and not any(isinstance(a, Navigate) for a in out)
    trip.replan_from("entrance:aq_convocation_mall")  # the sign says the car is at the entrance
    assert trip.path[0] == "entrance:aq_convocation_mall"
    out = trip.on_localization(True)
    nav = [a for a in out if isinstance(a, Navigate)]
    assert nav and nav[0].nodes[0] == "entrance:aq_convocation_mall"


def test_regained_localization_respects_pause_and_follow(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    trip.on_localization(False)
    out = trip.on_pause(True)
    assert any(isinstance(a, Recovery) and not a.on for a in out)  # stop rotating while paused
    out = trip.on_localization(True)
    assert not any(isinstance(a, Navigate) for a in out) and trip.state == "waiting_for_student"
    out = trip.on_pause(False)
    assert any(isinstance(a, Navigate) for a in out)
    trip.on_localization(False)
    trip.on_follow(False)
    out = trip.on_localization(True)
    assert not any(isinstance(a, Navigate) for a in out)


def test_localization_lost_while_waiting_is_not_forgotten(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    trip.on_follow(False)
    trip.on_localization(False)
    out = trip.on_follow(True)
    assert trip.state == "lost" and not any(isinstance(a, Navigate) for a in out)


def test_confirms_respect_pause(campus):
    trip = new_trip(campus)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    trip.on_nav_result(False)
    trip.on_pause(True)
    out = trip.on_confirm()
    assert not any(isinstance(a, Navigate) for a in out) and trip.state == "waiting_for_student"
    assert any(isinstance(a, Navigate) for a in trip.on_pause(False))


def test_new_trip_resets_waiting_for(campus):
    trip = new_trip(campus, voice_mode=True)
    trip.start("AQ 3150", "entrance:aq_convocation_mall")
    trip.waiting_for = "still_with_me"
    trip.abort("x")
    out = trip.start("AQ 3150", "entrance:aq_convocation_mall")
    assert trip.waiting_for == "" and any(isinstance(a, Navigate) for a in out)
