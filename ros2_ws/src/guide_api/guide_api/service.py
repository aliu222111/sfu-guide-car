"""The rider app's trips, on top of the mission node's topics (no ROS here).

One car, one trip at a time. A trip has up to two stages:

  summon  the car drives alone to where the rider is  (``/guide/destination`` = pickup room)
  guide   the car leads the rider to the destination  (``/guide/destination`` = destination room)

The service sends the mission its five inputs through a ``CarLink`` and is fed
the mission's three outputs (``on_state``, ``on_route``, ``on_speech``). Every
change becomes one event for the app: phase, progress, time left, what the car
needs from the rider, where the car is on the app's own floor plan.
"""

from __future__ import annotations

import itertools
import math
import queue
import threading
import time
from dataclasses import dataclass, field

from . import progress
from .frames import AppFrames
from .phases import ACTIVE, FINAL, app_phase

BUILDINGS = ("WMC", "MBC", "ASB", "AQ")  # as sfu_map.route, longest first
ENDED = frozenset({"done", "aborted", "cancelled", "failed"})


def normalise_key(text: str) -> str:
    """"asb9896", "ASB-9896", " asb  9896 " -> "ASB 9896" (the destination index's key form)."""
    t = " ".join(str(text).upper().split())
    for b in BUILDINGS:
        if t.startswith(b):
            rest = t[len(b):].lstrip(" -")
            if rest:
                return f"{b} {rest}"
    return t


class CarLink:
    """What the service needs from the car side. The ROS node, and the fake car, implement it."""

    def send_destination(self, key: str) -> None: raise NotImplementedError
    def send_confirm(self) -> None: raise NotImplementedError
    def send_cancel(self) -> None: raise NotImplementedError
    def send_pause(self, paused: bool) -> None: raise NotImplementedError
    def send_heartbeat(self) -> None: raise NotImplementedError


class ApiError(Exception):
    def __init__(self, status: int, code: str, message: str):
        super().__init__(message)
        self.status, self.code, self.message = status, code, message


@dataclass
class Trip:
    id: str
    destination: str
    pickup: str | None
    stage: str                 # "summon" | "guide"
    phase: str = "requested"   # requested, enroute, arrived, guiding, help, waiting, lost, done, aborted, cancelled, failed
    seen_active: bool = False  # the mission has taken up the destination we sent for this stage
    sent_at: float = 0.0
    prompt: dict | None = None
    progress: float = 0.0
    eta_s: float | None = None
    left_m: float | None = None
    paused: bool = False
    reason: str = ""
    subscribers: list = field(default_factory=list)


class ScoutService:
    def __init__(self, link: CarLink, destinations: dict | None = None, frames: AppFrames | None = None, *,
                 clock=time.monotonic, speed: float = 0.5, ride_s: float = 60.0, car_id: str = "scout-01",
                 car_name: str = "Scout", validate: bool = True, state_timeout: float = 3.0, accept_timeout: float = 8.0):
        self.link = link
        self.destinations = destinations or {}
        self.frames = frames or AppFrames()
        self.clock = clock
        self.speed, self.ride_s = speed, ride_s
        self.car_id, self.car_name = car_id, car_name
        self.validate = validate and bool(self.destinations)
        self.state_timeout, self.accept_timeout = state_timeout, accept_timeout
        self.lock = threading.RLock()
        self.mission: dict = {}
        self.mission_at: float | None = None
        self.route: list[dict] = []
        self.last_speech = ""
        self.trip: Trip | None = None
        self._ids = itertools.count(1)

    # --- read side ----------------------------------------------------------
    def online(self) -> bool:
        return self.mission_at is not None and self.clock() - self.mission_at <= self.state_timeout

    def _place(self, floor, pose) -> dict:
        """The car's position, in the map frame and (when the building has a transform) on the app's plan."""
        out = {"floor": floor, "pose": None, "app": None}
        if pose:
            x, y, yaw = (list(pose) + [0.0, 0.0, 0.0])[:3]
            out["pose"] = {"x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 3)}
            p = self.frames.point(floor, x, y)
            if p:
                ahead = self.frames.point(floor, x + math.cos(yaw), y + math.sin(yaw))
                p["heading"] = round(math.degrees(math.atan2(ahead["y"] - p["y"], ahead["x"] - p["x"])), 1)
                out["app"] = p
        return out

    def car(self) -> dict:
        with self.lock:
            m = self.mission
            busy = bool(self.trip and self.trip.phase not in ENDED) or m.get("state") in ACTIVE
            status = "offline" if not self.online() else "busy" if busy else "available"
            return {"id": self.car_id, "name": self.car_name, "status": status, "battery": None,
                    "mission": {"state": m.get("state"), "detail": m.get("detail", ""), "destination": m.get("destination")},
                    **self._place(m.get("floor"), m.get("pose"))}

    def health(self) -> dict:
        return {"ok": True, "carOnline": self.online(), "destinations": len(self.destinations),
                "frames": sorted(self.frames.to_map), "trip": self.trip.id if self.trip and self.trip.phase not in ENDED else None}

    def destination_list(self, q: str = "", limit: int = 40) -> list[dict]:
        n = "".join(q.upper().split()).replace("-", "")
        out = []
        for key, d in self.destinations.items():
            if n and n not in key.replace(" ", "").replace("-", "") and q.upper() not in (d.get("name") or "").upper():
                continue
            out.append({"key": key, "type": d.get("type", ""), "name": d.get("name", ""), "floor": d.get("floor", ""),
                        "floorName": d.get("floor_name", ""), "searchable": bool(d.get("searchable"))})
            if len(out) >= limit:
                break
        return out

    def _legs(self) -> list[dict] | None:
        """The route as the app draws it: one path per floor, in app coordinates."""
        legs: list[dict] = []
        for n in self.route:
            p = self.frames.point(n.get("floor"), n["x"], n["y"])
            if p is None:
                return None
            if not legs or legs[-1]["lv"] != p["lv"]:
                legs.append({"lv": p["lv"], "path": []})
            legs[-1]["path"].append([p["x"], p["y"]])
        return legs or None

    def snapshot(self, trip: Trip, *, say: str = "", with_route: bool = False) -> dict:
        m = self.mission
        ev = {"tripId": trip.id, "stage": trip.stage, "phase": trip.phase, "destination": trip.destination, "pickup": trip.pickup,
              "progress": round(trip.progress, 3), "etaSec": trip.eta_s, "leftM": trip.left_m, "paused": trip.paused,
              "online": self.online(), "prompt": trip.prompt, "reason": trip.reason, "say": say,
              **self._place(m.get("floor"), m.get("pose"))}
        if with_route:
            ev["route"] = self.route
            ev["legs"] = self._legs()
        return ev

    # --- the app's commands ---------------------------------------------------
    def _key(self, text: str, what: str) -> str:
        key = normalise_key(text or "")
        if not key:
            raise ApiError(400, "bad_request", f"{what} is missing")
        if self.validate and key not in self.destinations:
            raise ApiError(404, "unknown_room", f"{key} is not on the car's map")
        return key

    def _current(self, trip_id: str) -> Trip:
        t = self.trip
        if t is None or t.id != trip_id:
            raise ApiError(404, "unknown_trip", "no such trip")
        return t

    def _send(self, trip: Trip, key: str):
        trip.seen_active = False
        trip.sent_at = self.clock()
        trip.prompt, trip.progress, trip.eta_s, trip.left_m = None, 0.0, None, None
        self.route = []
        self.link.send_destination(key)

    def request_trip(self, destination: str, pickup: str | None = None) -> dict:
        """Start a trip. With ``pickup`` the car first drives there alone; without, the rider is already at the car."""
        with self.lock:
            dest = self._key(destination, "destination")
            pick = self._key(pickup, "pickup") if pickup else None
            if self.trip and self.trip.phase not in ENDED:
                raise ApiError(409, "busy", "the car is on another trip")
            if not self.online():
                raise ApiError(503, "offline", "the car is not answering")
            if self.mission.get("state") in ACTIVE:
                raise ApiError(409, "busy", "the car is on a trip started somewhere else")
            trip = Trip(id=f"trip-{next(self._ids)}", destination=dest, pickup=pick, stage="summon" if pick else "guide")
            self.trip = trip
            self._send(trip, pick or dest)
            return self.snapshot(trip)

    def start_guiding(self, trip_id: str) -> dict:
        """The rider has met the car: now head for the destination."""
        with self.lock:
            trip = self._current(trip_id)
            if trip.stage != "summon" or trip.phase != "arrived":
                raise ApiError(409, "not_ready", "the car has not reached the rider yet")
            trip.stage, trip.phase = "guide", "requested"
            self._send(trip, trip.destination)
            return self._emit(trip)

    def confirm(self, trip_id: str) -> dict:
        """The rider did what the car asked (held the door, called the elevator, is still there)."""
        with self.lock:
            trip = self._current(trip_id)
            self.link.send_confirm()
            return self.snapshot(trip)

    def pause(self, trip_id: str, paused: bool) -> dict:
        with self.lock:
            trip = self._current(trip_id)
            self.link.send_pause(bool(paused))
            return self.snapshot(trip)

    def cancel(self, trip_id: str) -> dict:
        with self.lock:
            trip = self._current(trip_id)
            if trip.phase not in ENDED:
                self.link.send_cancel()
                trip.phase, trip.prompt, trip.reason = "cancelled", None, "cancelled"
                return self._emit(trip)
            return self.snapshot(trip)

    def relocate(self, room: str, heading_deg: float | None = None) -> dict:
        """Tell a car that cannot localize itself where it is standing (the straight-driving car)."""
        with self.lock:
            key = self._key(room, "room")
            setter = getattr(self.link, "set_position", None)
            if setter is None:
                raise ApiError(409, "not_supported", "this car finds its own position")
            if not setter(key, None if heading_deg is None else math.radians(float(heading_deg))):
                raise ApiError(409, "busy", "the car is on a trip")
            return {"room": key}

    # --- events to the app ------------------------------------------------------
    def subscribe(self, trip_id: str) -> tuple[queue.Queue, dict]:
        with self.lock:
            trip = self._current(trip_id)
            q: queue.Queue = queue.Queue(maxsize=200)
            trip.subscribers.append(q)
            return q, self.snapshot(trip, with_route=True)

    def unsubscribe(self, q: queue.Queue):
        with self.lock:
            if self.trip and q in self.trip.subscribers:
                self.trip.subscribers.remove(q)

    def _emit(self, trip: Trip, *, say: str = "", with_route: bool = False) -> dict:
        ev = self.snapshot(trip, say=say, with_route=with_route)
        for q in list(trip.subscribers):
            try:
                q.put_nowait(ev)
            except queue.Full:  # a stalled client: drop it rather than block the car's callbacks
                trip.subscribers.remove(q)
        return ev

    # --- fed by the car side ------------------------------------------------------
    def on_state(self, doc: dict):
        with self.lock:
            self.mission = doc
            self.mission_at = self.clock()
            trip = self.trip
            if trip is None or trip.phase in ENDED:
                return
            state, detail = doc.get("state", ""), doc.get("detail", "")
            if state in ACTIVE:
                trip.seen_active = True
            elif state in FINAL and not trip.seen_active:
                return  # left over from the trip before: the mission has not taken up ours yet
            phase, prompt = app_phase(trip.stage, state, detail)
            if phase is None:
                return
            trip.phase = phase
            trip.prompt = prompt.to_dict() if prompt else None
            if trip.prompt and doc.get("instruction"):
                trip.prompt["line"] = doc["instruction"]   # the car's own words for this stop ("Turn left about 90 degrees ...")
            trip.paused = bool(doc.get("paused"))
            est = progress.estimate(self.route, doc.get("pose"), doc.get("floor"), self.speed, self.ride_s)
            if phase in ("arrived", "done"):
                trip.progress, trip.eta_s, trip.left_m = 1.0, 0.0, 0.0
            elif est:
                trip.progress, trip.eta_s, trip.left_m = est["progress"], est["etaSec"], est["leftM"]
            if phase == "aborted":
                trip.reason = detail or "aborted"
            self._emit(trip)

    def on_route(self, nodes: list[dict]):
        with self.lock:
            self.route = nodes or []
            if self.trip and self.trip.phase not in ENDED:
                self._emit(self.trip, with_route=True)

    def on_speech(self, text: str):
        with self.lock:
            self.last_speech = text
            if self.trip and self.trip.phase not in ENDED:
                self._emit(self.trip, say=text)

    def tick(self):
        """Call about once a second: the follow heartbeat, and a trip the mission never took up."""
        with self.lock:
            trip = self.trip
            if trip is None or trip.phase in ENDED:
                return
            if trip.subscribers:
                self.link.send_heartbeat()  # an open app counts as "the rider is here" (spec section 9)
            if not trip.seen_active and self.clock() - trip.sent_at > self.accept_timeout:
                trip.phase, trip.prompt = "failed", None
                trip.reason = self.last_speech or "The car did not start the trip."
                self._emit(trip)
