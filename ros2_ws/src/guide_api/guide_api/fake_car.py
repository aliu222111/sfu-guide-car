"""A stand-in for the mission node, so the API and the app can be worked on without the car or ROS.

It answers the same five inputs and produces the same ``/guide/state``,
``/guide/route`` and ``/guide/speech`` documents, driving in a straight line to
the room at walking pace. Between floors it asks for the elevator the way the
real mission does (``elevator_call``, then ``elevator_ride``); with ``door=True``
it also stops once for a door. It knows nothing about walls: use the simulator
(``sim.launch.py``) when the route matters.
"""

from __future__ import annotations

import math
import threading

from .service import CarLink, normalise_key


class FakeCar(CarLink):
    def __init__(self, locate=None, start=("ASB-09", 0.0, 0.0), speed: float = 0.5, door: bool = False):
        self.locate = locate or (lambda key: None)   # key -> (floor, x, y) | None
        self.floor, self.x, self.y = start
        self.yaw = 0.0
        self.speed = speed
        self.door = door
        self.service = None
        self.lock = threading.Lock()
        self.state, self.detail, self.destination = "idle", "", None
        self.paused = False
        self.path: list[dict] = []
        self.i = 0                 # index of the node being driven to
        self.timer = 0.0
        self.door_done = False
        self._out: list[tuple] = []

    def attach(self, service):
        self.service = service

    # --- CarLink ------------------------------------------------------------
    def send_destination(self, key: str):
        with self.lock:
            key = normalise_key(key)
            where = self.locate(key)
            if where is None:
                self._out.append(("speech", f"I can't find {key} on my map."))
                return
            floor, x, y = where
            here = {"id": "fake:start", "x": self.x, "y": self.y, "floor": self.floor, "type": "corridor_junction"}
            path = [here]
            if floor != self.floor:  # ride from where the car is
                path.append({"id": "fake:elevator:a", "x": self.x + 3.0, "y": self.y, "floor": self.floor, "type": "elevator"})
                path.append({"id": "fake:elevator:b", "x": self.x + 3.0, "y": self.y, "floor": floor, "type": "elevator"})
            path.append({"id": f"fake:door:{key}", "x": x, "y": y, "floor": floor, "type": "door"})
            self.path, self.i = path, 1
            self.destination, self.paused, self.door_done = key, False, False
            self.state, self.detail, self.timer = "routing", "", 0.6
            self._out.append(("speech", f"Planning the way to {key}."))

    def send_confirm(self):
        with self.lock:
            if self.state != "waiting_for_help":
                return
            if self.detail == "elevator_call":
                self.detail = "elevator_ride"
                self._out.append(("speech", "Thank you. Please press the floor button and tell me when we are there."))
            elif self.detail == "elevator_ride":
                nxt = self.path[self.i]
                self.floor, self.x, self.y = nxt["floor"], nxt["x"], nxt["y"]
                self.i += 1
                self.state, self.detail = "driving", ""
            else:
                self.state, self.detail = "driving", ""

    def send_cancel(self):
        with self.lock:
            if self.state not in ("idle", "arrived", "aborted"):
                self.state, self.detail = "aborted", "cancelled"

    def send_pause(self, paused: bool):
        with self.lock:
            self.paused = bool(paused)

    def send_heartbeat(self):
        pass

    # --- the fake mission ---------------------------------------------------
    def _doc(self) -> dict:
        return {"state": self.state, "detail": self.detail, "destination": self.destination, "floor": self.floor,
                "leg": 0, "legs": 1, "leg_kind": "drive" if self.state == "driving" else None,
                "paused": self.paused, "pose": [round(self.x, 3), round(self.y, 3), round(self.yaw, 3)]}

    def step(self, dt: float):
        """Advance ``dt`` seconds and feed the service, as the mission's timers would."""
        with self.lock:
            if self.state == "routing":
                self.timer -= dt
                if self.timer <= 0:
                    self.state = "driving"
                    self._out.append(("route", [dict(n) for n in self.path]))
                    self._out.append(("speech", "Follow me."))
            elif self.state == "driving" and not self.paused:
                self._drive(dt)
            out, self._out = self._out, []
            doc = self._doc()
        if self.service is None:
            return
        for kind, value in out:  # outside our lock: the service calls back into send_*
            (self.service.on_route if kind == "route" else self.service.on_speech)(value)
        self.service.on_state(doc)

    def _drive(self, dt: float):
        left = self.speed * dt
        while left > 0 and self.i < len(self.path):
            node = self.path[self.i]
            if node["floor"] != self.floor:
                self.state, self.detail = "waiting_for_help", "elevator_call"
                self._out.append(("speech", "Could you call the elevator and hold the doors?"))
                return
            d = math.hypot(node["x"] - self.x, node["y"] - self.y)
            if self.door and not self.door_done and self.i == len(self.path) - 1 and d < 4.0:
                self.door_done = True
                self.state, self.detail = "waiting_for_help", "door"
                self._out.append(("speech", "Could you hold the door for me?"))
                return
            if d > 1e-6:
                self.yaw = math.atan2(node["y"] - self.y, node["x"] - self.x)
            if d <= left:
                self.x, self.y = node["x"], node["y"]
                left -= d
                self.i += 1
            else:
                self.x += (node["x"] - self.x) * left / d
                self.y += (node["y"] - self.y) * left / d
                left = 0
        if self.i >= len(self.path):
            self.state, self.detail = "arrived", ""
            self._out.append(("speech", f"We are at {self.destination}."))
