"""The trip state machine (spec section 8), free of ROS so it can be tested alone.

The mission node feeds it events and carries out the actions it returns.

States: idle, routing, driving, waiting_for_student, waiting_for_help, lost,
arrived, aborted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from sfu_map.route import Leg, RouteError, RouteGraph

from .instructions import Turn, arrival_text, find_turns, floor_label, turn_text

# --- Actions the node carries out --------------------------------------------


@dataclass
class Speak:
    text: str


@dataclass
class Navigate:
    """Drive through these graph nodes with Nav2 on ``floor`` (None outdoors)."""

    floor: str | None
    nodes: list[str]


@dataclass
class CancelNav:
    pass


@dataclass
class SetLocalization:
    """Switch the localization setup: outdoor (GPS fused) or indoor on a floor."""

    floor: str | None


@dataclass
class SetPose:
    """Tell the localizer where the car is (after an elevator ride)."""

    node: str
    sigma: float


@dataclass
class Creep:
    """Drive straight for a short distance without Nav2 (into or out of an elevator)."""

    distance: float
    speed: float


@dataclass
class Recovery:
    on: bool


@dataclass
class DriveEnable:
    on: bool


@dataclass
class StateChanged:
    state: str
    detail: str = ""


# --- Parameters ----------------------------------------------------------------


@dataclass
class TripParams:
    announce_ahead: float = 5.0  # m, spec section 8
    elevator_creep: float = 1.5  # m into and out of the car
    creep_speed: float = 0.25  # m/s
    voice_mode: bool = False  # no phone: "still with me?" at each turn (spec section 9)
    elevator_sigma: float = 1.5  # m, pose uncertainty on leaving an elevator
    passed_radius: float = 1.5  # m, a node counts as reached within this distance


@dataclass
class _Elevator:
    stage: str  # call, board, ride, exit, relocalize
    leg: Leg


@dataclass
class Trip:
    rg: RouteGraph
    params: TripParams = field(default_factory=TripParams)
    state: str = "idle"
    destination: str = ""
    goal: str = ""
    path: list[str] = field(default_factory=list)
    legs: list[Leg] = field(default_factory=list)
    leg_index: int = 0
    progress: int = 0  # index into the current drive leg's nodes reached so far
    floor: str | None = None  # where localization is set up now
    paused: bool = False
    following: bool = True
    localized: bool = True
    turns: list[Turn] = field(default_factory=list)
    announced: set = field(default_factory=set)
    elevator: _Elevator | None = None
    waiting_for: str = ""  # what a confirm means in waiting_for_help / waiting_for_student

    # --- Helpers ---------------------------------------------------------------
    def _set(self, state: str, detail: str = "") -> list:
        self.state = state
        return [StateChanged(state, detail)]

    @property
    def leg(self) -> Leg | None:
        return self.legs[self.leg_index] if 0 <= self.leg_index < len(self.legs) else None

    @property
    def active(self) -> bool:
        return self.state not in ("idle", "arrived", "aborted")

    # --- Events ----------------------------------------------------------------
    def start(self, destination: str, start_node: str) -> list:
        """Plan from ``start_node`` to a room or node and begin driving."""
        out = []
        if self.active:
            out += self.abort("new destination")
        self.destination = destination
        out += self._set("routing", destination)
        try:
            self.goal = self.rg.resolve(destination)
            path, length = self.rg.shortest_path(start_node, self.goal)
        except RouteError as e:
            out += self._set("aborted", str(e))
            out.append(Speak(f"Sorry, I can't find a way to {destination}."))
            return out
        self.path = path
        self.legs = self.rg.legs(path) if len(path) > 1 else []
        self.leg_index = 0
        self.turns = find_turns(self.rg, path)
        self.announced = set()
        self.elevator = None
        out.append(Speak(f"Let's go to {destination}. Follow me, it's about {round_distance(length)}."))
        out.append(DriveEnable(True))
        return out + self._begin_leg()

    def _begin_leg(self) -> list:
        leg = self.leg
        if leg is None:
            return self._arrive()
        self.progress = 0
        out = []
        if leg.kind == "drive":
            if leg.floor != self.floor:
                out.append(SetLocalization(leg.floor))
                self.floor = leg.floor
            out += self._set("driving", leg.floor or "outdoors")
            if self.paused or not self.following:
                return out + self._hold()
            out.append(Navigate(leg.floor, list(leg.nodes)))
            return out
        if leg.kind == "door":
            self.waiting_for = "door"
            out += self._set("waiting_for_help", "door")
            out.append(Speak("Please hold the door for me, and tell me when it's open."))
            return out
        if leg.kind == "elevator":
            self.elevator = _Elevator("call", leg)
            self.waiting_for = "elevator"
            target = self.rg.nodes[leg.nodes[-1]]["floor"]
            here = self.rg.nodes[leg.nodes[0]]["floor"]
            direction = "up" if self._level(target) > self._level(here) else "down"
            out += self._set("waiting_for_help", "elevator_call")
            out.append(Speak(f"We'll take the elevator {direction} to {floor_label(self.rg, target)}. Please press the call button and tell me when the doors open."))
            return out
        # Stairs never appear in a car route; treat anything else as unroutable.
        return self.abort(f"cannot handle a {leg.kind} leg")

    def _level(self, floor):
        meta = self.rg.floors.get(floor, {})
        return meta.get("level", 0) or 0

    def _hold(self) -> list:
        out = [CancelNav()]
        if self.paused:
            out += self._set("waiting_for_student", "paused")
        else:
            out += self._set("waiting_for_student", "follow")
            out.append(Speak("I'll wait for you."))
        return out

    def on_nav_result(self, success: bool) -> list:
        if self.state != "driving":
            return []  # a cancelled goal reporting back
        if not success:
            # Nav2 gave up (blocked corridor). Wait for the student to clear it
            # or pick another destination rather than guessing.
            self.waiting_for = "blocked"
            out = self._set("waiting_for_help", "blocked")
            out.append(Speak("I'm stuck. Could you check the way ahead is clear and tell me when to try again?"))
            return out
        self.leg_index += 1
        return self._begin_leg()

    def on_confirm(self) -> list:
        """The student pressed a button on the phone or said yes."""
        if self.state == "waiting_for_student" and self.waiting_for == "still_with_me":
            self.waiting_for = ""
            return self._resume()
        if self.state != "waiting_for_help":
            return []
        if self.waiting_for == "blocked":
            self.waiting_for = ""
            return self._resume()
        if self.waiting_for == "door":
            # Drive through the door edge as a short drive leg.
            self.waiting_for = ""
            leg = self.leg
            out = self._set("driving", "door")
            out.append(Navigate(self.rg.nodes[leg.nodes[-1]]["floor"], list(leg.nodes)))
            return out
        if self.waiting_for == "elevator" and self.elevator:
            el = self.elevator
            target = self.rg.nodes[el.leg.nodes[-1]]["floor"]
            if el.stage == "call":
                el.stage = "board"
                out = self._set("waiting_for_help", "elevator_board")
                out.append(Speak("Please hold the doors. I'm coming in."))
                out.append(Creep(self.params.elevator_creep, self.params.creep_speed))
                return out
            if el.stage == "ride":
                el.stage = "exit"
                self.floor = target
                out = self._set("waiting_for_help", "elevator_exit")
                out.append(Speak("Thanks. Please hold the doors while I get out."))
                out.append(SetLocalization(target))
                out.append(Creep(self.params.elevator_creep, self.params.creep_speed))
                return out
        return []

    def on_creep_done(self) -> list:
        el = self.elevator
        if self.state != "waiting_for_help" or not el:
            return []
        if el.stage == "board":
            el.stage = "ride"
            target = self.rg.nodes[el.leg.nodes[-1]]["floor"]
            out = self._set("waiting_for_help", "elevator_ride")
            out.append(Speak(f"Please press {floor_label(self.rg, target)}, and tell me when the doors open there."))
            return out
        if el.stage == "exit":
            el.stage = "relocalize"
            out = [SetPose(el.leg.nodes[-1], self.params.elevator_sigma)]
            out += self._set("lost", "after elevator")
            self.localized = False
            out.append(Recovery(True))
            return out
        return []

    def on_localization(self, ok: bool) -> list:
        if ok == self.localized:
            return []
        self.localized = ok
        if not self.active:
            return []
        if not ok:
            if self.state == "driving":
                out = [CancelNav(), Recovery(True)] + self._set("lost", "localization lost")
                out.append(Speak("I've lost track of where I am. One moment."))
                return out
            return []
        if self.state == "lost":
            out = [Recovery(False)]
            if self.elevator and self.elevator.stage == "relocalize":
                self.elevator = None
                self.leg_index += 1
                return out + self._begin_leg()
            return out + self._resume()
        return []

    def on_follow(self, following: bool) -> list:
        if following == self.following:
            return []
        self.following = following
        if not self.active:
            return []
        if not following and self.state == "driving":
            return self._hold()
        if following and self.state == "waiting_for_student" and not self.paused and self.waiting_for != "still_with_me":
            out = [Speak("There you are. Let's keep going.")]
            return out + self._resume()
        return []

    def on_pause(self, paused: bool) -> list:
        if paused == self.paused:
            return []
        self.paused = paused
        if not self.active:
            return []
        if paused and self.state == "driving":
            return self._hold()
        if not paused and self.state == "waiting_for_student" and self.following:
            return self._resume()
        return []

    def _resume(self) -> list:
        leg = self.leg
        if leg is None:
            return self._arrive()
        if leg.kind != "drive":
            return self._begin_leg()
        remaining = leg.nodes[self.progress:]
        out = self._set("driving", leg.floor or "outdoors")
        out.append(Navigate(leg.floor, remaining if len(remaining) > 1 else leg.nodes[-2:]))
        return out

    def on_position(self, x: float, y: float) -> list:
        """Track progress along the current drive leg and announce turns."""
        leg = self.leg
        if self.state != "driving" or leg is None or leg.kind != "drive":
            return []
        nodes = leg.nodes
        for i in range(self.progress, len(nodes)):
            n = self.rg.nodes[nodes[i]]
            if math.hypot(n["x"] - x, n["y"] - y) <= self.params.passed_radius:
                self.progress = i
        out = []
        for t in self.turns:
            if t.node in self.announced or t.node not in nodes[self.progress:]:
                continue
            n = self.rg.nodes[t.node]
            d = math.hypot(n["x"] - x, n["y"] - y)
            if d <= self.params.announce_ahead:
                self.announced.add(t.node)
                out.append(Speak(turn_text(t, d)))
                if self.params.voice_mode:
                    self.waiting_for = "still_with_me"
                    out.append(CancelNav())
                    out += self._set("waiting_for_student", "still_with_me")
                    out.append(Speak("Still with me?"))
                    return out
        return out

    def _arrive(self) -> list:
        out = [CancelNav(), DriveEnable(False)] + self._set("arrived", self.destination)
        out.append(Speak(arrival_text(self.rg, self.path, self.destination)))
        return out

    def abort(self, reason: str = "") -> list:
        if not self.active:
            return []
        self.elevator = None
        out = [CancelNav(), Recovery(False), DriveEnable(False)] + self._set("aborted", reason)
        return out


def round_distance(m: float) -> str:
    if m < 100:
        return f"{int(round(m / 10.0) * 10) or 10} metres"
    minutes = m / 1.2 / 60  # a relaxed walking pace
    return f"{int(round(m / 50.0) * 50)} metres, {max(1, round(minutes))} minutes on foot"
