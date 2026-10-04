"""The straight-driving car: planner, legs, hand turns, the board's serial line."""

import math
from pathlib import Path

from guide_api.frames import AppFrames
from guide_api.planner import Leg, Planner, turn_text
from guide_api.service import ScoutService
from guide_api.straight_car import StraightCar, drive_line

CONFIG = Path(__file__).resolve().parents[1] / "guide_api" / "config"
FRAMES = AppFrames.load(CONFIG / "app_frames.json")
PLANNER = Planner(CONFIG / "asb_plan.json", FRAMES.to_map["ASB"])


class Board:
    """Stands in for the ESP32: remembers what it was sent, can press its button."""

    def __init__(self):
        self.sent: list[bytes] = []
        self.inbox = b""

    def write(self, data: bytes):
        self.sent.append(data)

    def read(self) -> bytes:
        out, self.inbox = self.inbox, b""
        return out

    def throttle(self) -> float:
        return float(self.sent[-1].split()[2])


def make(start="ASB 970.7", speed=1.0):
    board = Board()
    car = StraightCar(PLANNER, board, start=PLANNER.locate(start), throttle=0.6, speed=speed)
    t = [0.0]
    service = ScoutService(car, PLANNER.destinations(), FRAMES, clock=lambda: t[0])
    car.attach(service)

    def run(seconds, dt=0.05):
        for _ in range(int(round(seconds / dt))):
            t[0] += dt
            car.step(dt)
        service.tick()

    run(0.1)
    return service, car, board, run


def test_turn_text():
    assert turn_text(None, 1.0)[0] == "point"
    assert turn_text(0.0, math.radians(90))[0] == "left"
    assert turn_text(0.0, math.radians(-90))[0] == "right"
    assert turn_text(0.0, math.radians(175))[0] == "around"
    assert turn_text(math.radians(175), math.radians(-178))[0] == "ahead"  # across the wrap: 7 degrees, not 353
    assert turn_text(math.radians(170), math.radians(-160))[0] == "left"   # 30 degrees counter-clockwise
    assert turn_text(0.0, math.radians(5))[0] == "ahead"


def test_drive_line_clamps():
    assert drive_line(0.6) == b"C 0.000 0.600\n"
    assert drive_line(7) == b"C 0.000 1.000\n"
    assert drive_line(0.0) == b"C 0.000 0.000\n"


def test_plan_is_a_few_clear_straight_legs_that_join_up():
    floor, ax, ay = PLANNER.locate("ASB 970.7")
    _, bx, by = PLANNER.locate("ASB 9700")
    legs = PLANNER.plan(floor, (ax, ay), (bx, by))
    assert 1 <= len(legs) <= 8
    for a, b in zip(legs, legs[1:]):
        assert math.isclose(a.x1, b.x0) and math.isclose(a.y1, b.y0)
    assert math.dist((legs[0].x0, legs[0].y0), (ax, ay)) < 3.0
    assert math.dist((legs[-1].x1, legs[-1].y1), (bx, by)) < 6.0   # the room's label may sit off the walkable floor
    assert 50 < sum(l.length for l in legs) < 110                   # about 75 m through the concourse
    walk, clr = PLANNER.grid("9")
    for leg in legs:                                                 # every leg stays on walkable floor
        for i in range(101):
            x, y = PLANNER.to_plan.apply(leg.x0 + (leg.x1 - leg.x0) * i / 100, leg.y0 + (leg.y1 - leg.y0) * i / 100)
            assert walk[int(y // PLANNER.cell) * PLANNER.gw + int(x // PLANNER.cell)]


def test_summon_waits_to_be_pointed_then_drives_each_leg_for_its_length():
    service, car, board, run = make(speed=1.0)
    assert board.throttle() == 0.0                    # idle: the board is told to stay still
    trip = service.request_trip("ASB 9896", pickup="ASB 9896")
    run(0.2)
    assert car.state == "waiting_for_help" and car.detail == "turn"
    assert service.trip.phase == "help" and service.trip.prompt["kind"] == "turn"
    assert service.trip.prompt["line"].startswith("Point the car toward ASB ")
    assert board.throttle() == 0.0                    # not moving while it waits
    legs = list(car.legs)
    run(5)
    assert car.state == "waiting_for_help" and board.throttle() == 0.0   # still waiting: nobody confirmed

    service.confirm(trip["tripId"])
    run(0.1)
    assert car.state == "driving" and board.throttle() == 0.6
    assert service.trip.phase == "enroute"
    run(legs[0].length - 0.5)                          # 1 m/s: almost at the end of the first leg
    assert car.i == 0 and car.state == "driving"
    run(0.7)
    assert car.i == 1 and car.state == "waiting_for_help" and board.throttle() == 0.0
    assert "Turn the car" in car.instruction and "degrees" in car.instruction

    for n in range(1, len(legs)):
        board.inbox += b"B 1\nB 0\n"                   # the button on the car
        run(legs[n].length + 0.3)
    assert car.state == "arrived" and board.throttle() == 0.0
    assert service.trip.phase == "arrived"
    assert math.dist((car.x, car.y), (legs[-1].x1, legs[-1].y1)) < 1e-6
    assert math.isclose(car.heading, legs[-1].heading)


def test_pause_cancel_and_other_floor():
    service, car, board, run = make()
    trip = service.request_trip("ASB 9896")
    run(0.2)
    service.confirm(trip["tripId"])
    run(1)
    assert board.throttle() == 0.6
    service.pause(trip["tripId"], True)
    run(0.2)
    x = car.x
    run(2)
    assert board.throttle() == 0.0 and car.x == x      # paused: stopped and not counting distance
    service.pause(trip["tripId"], False)
    run(0.2)
    assert board.throttle() == 0.6
    service.cancel(trip["tripId"])
    run(0.2)
    assert car.state == "aborted" and board.throttle() == 0.0

    trip = service.request_trip("ASB 10900")           # another level
    run(0.3)
    assert service.trip.phase == "aborted" and service.trip.reason == "other_floor"


def test_something_ahead_stops_the_car_until_told_to_try_again():
    service, car, board, run = make()
    trip = service.request_trip("ASB 9896")
    run(0.2)
    service.confirm(trip["tripId"])
    run(0.5)
    board.inbox += b"U 25\n"
    run(0.2)
    assert car.detail == "blocked" and board.throttle() == 0.0 and service.trip.prompt["kind"] == "blocked"
    board.inbox += b"U 0\n"                             # clear again
    service.confirm(trip["tripId"])
    run(0.2)
    assert car.state == "driving" and board.throttle() == 0.6


def test_set_position_between_trips_only():
    service, car, board, run = make()
    assert service.relocate("asb 9700", 90) == {"room": "ASB 9700"}
    assert (car.floor, round(car.x, 1)) == ("ASB-09", round(PLANNER.locate("ASB 9700")[1], 1))
    assert math.isclose(car.heading, math.pi / 2)
    trip = service.request_trip("ASB 9896")
    run(0.2)
    try:
        service.relocate("ASB 9700")
    except Exception as e:
        assert getattr(e, "code", "") == "busy"
    else:
        raise AssertionError("expected busy")
