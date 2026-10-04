"""Trips against the fake car, and over HTTP."""

import json
import threading
import urllib.error
import urllib.request

from guide_api.fake_car import FakeCar
from guide_api.frames import Affine, AppFrames
from guide_api.http_api import serve
from guide_api.service import ApiError, ScoutService

ROOMS = {
    "ASB 9700": {"floor": "ASB-09", "room_x": 10.0, "room_y": 0.0},
    "ASB 9896": {"floor": "ASB-09", "room_x": 10.0, "room_y": 20.0},
    "ASB 10900": {"floor": "ASB-10", "room_x": 30.0, "room_y": 0.0},
}


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(door=False, validate=True):
    clock = Clock()
    car = FakeCar(lambda k: (ROOMS[k]["floor"], ROOMS[k]["room_x"], ROOMS[k]["room_y"]) if k in ROOMS else None,
                  start=("ASB-09", 0.0, 0.0), door=door)
    frames = AppFrames({"ASB": Affine(0.1, 0.0, 0.0, 0.0, -0.1, 0.0)})
    service = ScoutService(car, ROOMS, frames, clock=clock, validate=validate)
    car.attach(service)

    def run(seconds, dt=0.5):
        for _ in range(int(seconds / dt)):
            clock.t += dt
            car.step(dt)
            service.tick()

    run(1)  # the car reports in
    return service, car, run


def raises(code, fn, *a):
    try:
        fn(*a)
    except ApiError as e:
        assert e.code == code, e.code
        return
    raise AssertionError(f"expected {code}")


def test_car_is_available_then_busy():
    service, car, run = make()
    assert service.car()["status"] == "available"
    assert service.car()["app"] == {"lv": "9", "x": 0.0, "y": -0.0, "heading": -0.0}
    service.request_trip("ASB 9896")
    run(2)
    assert service.car()["status"] == "busy"


def test_summon_then_guide():
    service, car, run = make()
    trip = service.request_trip("asb9896", pickup="ASB 9700")
    assert trip["stage"] == "summon" and trip["phase"] == "requested"
    q, first = service.subscribe(trip["tripId"])
    run(3)
    assert service.trip.phase == "enroute" and 0 < service.trip.progress < 1 and service.trip.eta_s > 0
    raises("not_ready", service.start_guiding, trip["tripId"])
    run(30)
    assert service.trip.phase == "arrived" and service.trip.progress == 1.0
    service.start_guiding(trip["tripId"])
    assert service.trip.stage == "guide"
    run(1)   # still "arrived" from the summon: must not count as the end of the guide stage
    assert service.trip.phase in ("requested", "guiding")
    run(60)
    assert service.trip.phase == "done"
    phases = []
    while not q.empty():
        ev = q.get()
        if not phases or phases[-1] != ev["phase"]:
            phases.append(ev["phase"])
    # "requested" appears twice: once per stage, until the mission takes up that stage's destination
    assert phases == ["requested", "enroute", "arrived", "requested", "guiding", "done"], phases


def test_rider_already_at_the_car_skips_the_summon():
    service, car, run = make()
    trip = service.request_trip("ASB 9896")
    assert trip["stage"] == "guide"
    run(60)
    assert service.trip.phase == "done"


def test_door_and_elevator_need_a_confirm():
    service, car, run = make(door=True)
    trip = service.request_trip("ASB 10900")
    run(20)
    assert service.trip.phase == "help" and service.trip.prompt["kind"] == "elevator_call"
    service.confirm(trip["tripId"])
    run(1)
    assert service.trip.prompt["kind"] == "elevator_ride"
    service.confirm(trip["tripId"])
    run(120)
    assert service.trip.phase == "help" and service.trip.prompt["kind"] == "door"
    assert service.mission["floor"] == "ASB-10"
    service.confirm(trip["tripId"])
    run(30)
    assert service.trip.phase == "done"


def test_one_trip_at_a_time_and_cancel():
    service, car, run = make()
    trip = service.request_trip("ASB 9896")
    run(2)
    raises("busy", service.request_trip, "ASB 9700")
    service.cancel(trip["tripId"])
    assert service.trip.phase == "cancelled"
    run(2)
    assert car.state == "aborted" and service.trip.phase == "cancelled"
    assert service.request_trip("ASB 9700")["phase"] == "requested"  # free again


def test_unknown_room_and_offline():
    service, car, run = make()
    raises("unknown_room", service.request_trip, "ASB 1")
    raises("bad_request", service.request_trip, "")
    service.clock.t += 10  # nothing heard from the mission for 10 s
    assert service.car()["status"] == "offline"
    raises("offline", service.request_trip, "ASB 9896")


def test_a_trip_the_mission_never_takes_up_fails_with_what_the_car_said():
    service, car, run = make(validate=False)
    trip = service.request_trip("ASB 4242")  # the fake car has no such room
    run(12)
    assert service.trip.phase == "failed" and "ASB 4242" in service.trip.reason
    assert service.request_trip("ASB 9896")["tripId"] != trip["tripId"]


def test_heartbeat_only_while_an_app_is_listening():
    service, car, run = make()
    beats = []
    car.send_heartbeat = lambda: beats.append(1)
    trip = service.request_trip("ASB 9896")
    run(2)
    assert not beats
    q, _ = service.subscribe(trip["tripId"])
    run(2)
    assert beats
    service.unsubscribe(q)


def test_route_reaches_the_app_in_plan_coordinates():
    service, car, run = make()
    trip = service.request_trip("ASB 10900")
    q, _ = service.subscribe(trip["tripId"])
    run(3)
    events = []
    while not q.empty():
        events.append(q.get())
    legs = next(ev["legs"] for ev in events if ev.get("legs"))
    assert [leg["lv"] for leg in legs] == ["9", "10"]
    assert legs[1]["path"][-1] == [300.0, -0.0]


def _call(base, method, path, body=None):
    req = urllib.request.Request(base + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_http_round_trip_and_events():
    service, car, run = make()
    server = serve(service, port=0, host="127.0.0.1")
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert _call(base, "GET", "/api/health")[1]["carOnline"] is True
        assert _call(base, "GET", "/api/car")[1]["status"] == "available"
        assert [d["key"] for d in _call(base, "GET", "/api/destinations?q=98")[1]] == ["ASB 9896"]
        assert _call(base, "POST", "/api/trips", {"destination": "ASB 1"})[0] == 404
        code, trip = _call(base, "POST", "/api/trips", {"destination": "ASB 9896"})
        assert code == 201 and trip["phase"] == "requested"
        assert _call(base, "POST", "/api/trips/nope/cancel")[0] == 404

        seen = []

        def listen():
            with urllib.request.urlopen(f"{base}/api/trips/{trip['tripId']}/events", timeout=10) as r:
                for line in r:
                    if line.startswith(b"data: "):
                        seen.append(json.loads(line[6:])["phase"])

        t = threading.Thread(target=listen, daemon=True)
        t.start()
        while not service.trip.subscribers:
            pass
        run(60)
        t.join(5)
        assert not t.is_alive() and seen[0] == "requested" and "guiding" in seen and seen[-1] == "done"
    finally:
        server.shutdown()
