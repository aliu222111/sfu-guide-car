"""The car with two drive motors and no steering: straight legs, turned by hand in between (no ROS here).

It stands where the mission node would (the same ``/guide/state`` documents,
so the rider API and the app need no changes) and talks to the ESP32 +
L298N board (``guide_bridge/firmware/guide_l298n``) over its serial line.

For each leg of the planner's route it asks for the car to be turned to face
the leg, waits for a confirm (the app, or the button on the car), then drives
the leg's length. It has no wheel sensors: distance is time at a calibrated
speed, so the position is an estimate that drifts. Someone must walk with it.
"""

from __future__ import annotations

import math
import threading

from .planner import Leg, PlanError, Planner, turn_text
from .service import CarLink, normalise_key


class NullTransport:
    """No board attached: commands go nowhere. For trying the logic at a desk."""

    def write(self, data: bytes): pass
    def read(self) -> bytes: return b""


class SerialTransport:
    """The ESP32 on a serial port (pyserial)."""

    def __init__(self, port: str, baud: int = 115200):
        import serial  # pyserial; only needed with a real board

        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.2)

    def write(self, data: bytes):
        self.ser.write(data)

    def read(self) -> bytes:
        return self.ser.read(self.ser.in_waiting or 0)

    def close(self):
        self.ser.close()


def drive_line(throttle: float) -> bytes:
    """The board's command: steering (unused) and throttle, each -1..1."""
    return f"C 0.000 {max(-1.0, min(1.0, throttle)):.3f}\n".encode("ascii")


class StraightCar(CarLink):
    def __init__(self, planner: Planner, transport, *, start: tuple[str, float, float], heading: float | None = None,
                 throttle: float = 0.6, speed: float = 0.5, stop_m: float = 0.4):
        self.planner, self.transport = planner, transport
        self.floor, self.x, self.y = start
        self.heading = heading           # radians in the map frame; None until the car has been pointed once
        self.throttle = throttle         # what to send while driving
        self.speed = speed               # metres per second the car really does at that throttle (calibrate it)
        self.stop_m = stop_m             # ultrasonic: nearer than this ahead, stop and ask
        self.service = None
        self.lock = threading.Lock()
        self.state, self.detail, self.destination = "idle", "", None
        self.instruction = ""
        self.paused = False
        self.pending: str | None = None  # destination waiting to be planned
        self.legs: list[Leg] = []
        self.i = 0
        self.travelled = 0.0
        self.range_m = math.inf
        self._buf = b""
        self._out: list[tuple] = []

    def attach(self, service):
        self.service = service

    # --- CarLink --------------------------------------------------------------
    def send_destination(self, key: str):
        with self.lock:
            self.pending = normalise_key(key)
            self.destination = self.pending
            self.state, self.detail, self.instruction, self.paused = "routing", "", "", False
            self.legs, self.i, self.travelled = [], 0, 0.0

    def send_confirm(self):
        with self.lock:
            self._confirm()

    def send_cancel(self):
        with self.lock:
            if self.state not in ("idle", "arrived", "aborted"):
                self.state, self.detail, self.instruction, self.pending = "aborted", "cancelled", "", None

    def send_pause(self, paused: bool):
        with self.lock:
            self.paused = bool(paused)

    def send_heartbeat(self):
        pass

    def set_position(self, key: str, heading: float | None = None) -> bool:
        """Tell the car where it is (it cannot find out by itself). Only between trips."""
        with self.lock:
            where = self.planner.locate(normalise_key(key))
            if where is None or self.state in ("routing", "driving", "waiting_for_help"):
                return False
            self.floor, self.x, self.y = where
            self.heading = heading
            return True

    # --- the trip -------------------------------------------------------------
    def _confirm(self):
        if self.state != "waiting_for_help":
            return
        if self.detail == "turn":
            self.heading = self.legs[self.i].heading
        self.state, self.detail, self.instruction = "driving", "", ""

    def _plan(self):
        key, self.pending = self.pending, None
        where = self.planner.locate(key)
        if where is None:
            self._fail("unknown_room", f"I can't find {key} on my plan.")
            return
        floor, x, y = where
        if floor != self.floor:
            self._fail("other_floor", f"{key} is on another level. I can only drive on the level I am on.")
            return
        try:
            self.legs = self.planner.plan(floor, (self.x, self.y), (x, y))
        except PlanError as e:
            self._fail("no_route", f"I can't find a way to {key}: {e}.")
            return
        if not self.legs:
            self.state, self.detail = "arrived", ""
            self._out.append(("speech", f"I am already at {key}."))
            return
        nodes = [{"id": "leg:0", "x": self.legs[0].x0, "y": self.legs[0].y0, "floor": self.floor, "type": "corridor_junction"}]
        nodes += [{"id": f"leg:{n + 1}", "x": l.x1, "y": l.y1, "floor": self.floor, "type": "corridor_junction"} for n, l in enumerate(self.legs)]
        self._out.append(("route", nodes))
        self.i = 0
        self._begin_leg()

    def _fail(self, detail: str, text: str):
        self.state, self.detail = "aborted", detail
        self._out.append(("speech", text))

    def _begin_leg(self):
        leg = self.legs[self.i]
        self.x, self.y, self.travelled = leg.x0, leg.y0, 0.0
        kind, deg = turn_text(self.heading, leg.heading)
        if kind == "ahead":
            self.state, self.detail, self.instruction = "driving", "", ""
            return
        toward = self.planner.nearest_room(self.floor, leg.x1, leg.y1)
        if kind == "point":
            text = f"Point the car toward {toward}."
        elif kind == "around":
            text = f"Turn the car around, to face {toward}."
        else:
            text = f"Turn the car {kind} about {int(round(abs(deg) / 5.0) * 5)} degrees, toward {toward}."
        self.state, self.detail = "waiting_for_help", "turn"
        self.instruction = text + " Then press the button on the car."
        self._out.append(("speech", text))

    def _drive(self, dt: float):
        if self.range_m < self.stop_m:
            self.state, self.detail = "waiting_for_help", "blocked"
            self._out.append(("speech", "Something is in my way."))
            return
        leg = self.legs[self.i]
        self.travelled = min(leg.length, self.travelled + self.speed * dt)
        k = self.travelled / leg.length if leg.length else 1.0
        self.x, self.y = leg.x0 + (leg.x1 - leg.x0) * k, leg.y0 + (leg.y1 - leg.y0) * k
        if self.travelled >= leg.length:
            self.i += 1
            if self.i >= len(self.legs):
                self.state, self.detail = "arrived", ""
                self._out.append(("speech", f"We are at {self.destination}."))
            else:
                self._begin_leg()

    def _read_board(self):
        self._buf += self.transport.read()
        *lines, self._buf = self._buf.split(b"\n")
        if len(self._buf) > 128:
            self._buf = b""
        for raw in lines:
            parts = raw.decode("ascii", errors="replace").split()
            if len(parts) != 2:
                continue
            if parts[0] == "B" and parts[1] == "1":
                self._confirm()                      # the button on the car: same as the app's confirm
            elif parts[0] == "U":
                try:
                    cm = int(parts[1])
                except ValueError:
                    continue
                if cm >= 0:
                    self.range_m = math.inf if cm == 0 else cm / 100.0

    def _doc(self) -> dict:
        yaw = self.heading if self.heading is not None else 0.0
        return {"state": self.state, "detail": self.detail, "destination": self.destination, "floor": self.floor,
                "leg": self.i, "legs": len(self.legs), "leg_kind": "drive" if self.state == "driving" else None,
                "paused": self.paused, "pose": [round(self.x, 3), round(self.y, 3), round(yaw, 3)],
                "instruction": self.instruction}

    def step(self, dt: float):
        """Call at about 20 Hz: reads the board, moves the trip on, sends the board its command."""
        with self.lock:
            docs = []
            self._read_board()
            if self.state == "routing" and self.pending:
                docs.append(self._doc())             # let the API see "routing" before the plan lands
                self._plan()
            moving = self.state == "driving" and not self.paused
            if moving:
                self._drive(dt)
                moving = self.state == "driving"
            try:
                self.transport.write(drive_line(self.throttle if moving else 0.0))
            except Exception as e:                   # the cable came out: stop the trip rather than guess
                if self.state in ("routing", "driving", "waiting_for_help"):
                    self._fail("no_board", f"I lost the motor board: {e}")
            docs.append(self._doc())
            out, self._out = self._out, []
        if self.service is None:
            return
        for kind, value in out:                      # outside our lock: the service calls back into send_*
            (self.service.on_route if kind == "route" else self.service.on_speech)(value)
        for doc in docs:
            self.service.on_state(doc)
