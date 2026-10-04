"""Mission states -> what the rider app shows (no ROS here).

The mission node (guide_mission) publishes ``/guide/state`` with one of: idle,
routing, driving, waiting_for_student, waiting_for_help, lost, arrived, aborted.
The app has two stages the mission does not know about: the car driving to the
rider alone ("summon"), then guiding the rider ("guide"). This turns
(stage, state, detail) into the app's phase and, when the car needs something
from the rider, the words and the button for it.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

ACTIVE = frozenset({"routing", "driving", "waiting_for_student", "waiting_for_help", "lost"})
FINAL = frozenset({"arrived", "aborted"})


@dataclass(frozen=True)
class Prompt:
    kind: str
    title: str
    line: str
    button: str  # empty: nothing to press, just wait

    def to_dict(self) -> dict:
        return asdict(self)


# detail of waiting_for_help / waiting_for_student -> prompt. The button texts
# match the labels the car's own phone app uses (guide_ui/web/app.js).
PROMPTS = {
    "elevator_call": Prompt("elevator_call", "Call the elevator", "Press the button and hold the doors while the car drives in.", "The doors are open"),
    "elevator_ride": Prompt("elevator_ride", "Ride with the car", "Press the floor button. Tell the car when you are there and the doors are open.", "We're there, doors open"),
    "door": Prompt("door", "Hold the door", "The car cannot open doors. Hold this one so it can drive through.", "I'm holding the door"),
    "blocked": Prompt("blocked", "The way is blocked", "Something is in the car's path. Clear it, or wait for it to move.", "Try again"),
    # The straight-driving car (straight_car.py) cannot steer: someone turns it by hand between legs.
    "turn": Prompt("turn", "Turn the car", "Swing the car to face the next stretch, then confirm.", "It's turned"),
    "still_with_me": Prompt("still_with_me", "Still with me?", "The car lost sight of you and stopped.", "Yes, I'm here"),
}
LOST = Prompt("lost", "Finding where I am", "The car is working out its position. Stay close.", "")
WAITING = Prompt("waiting", "Waiting for you", "The car stopped so you can catch up.", "")


def app_phase(stage: str, state: str, detail: str = "") -> tuple[str | None, Prompt | None]:
    """The app's phase for a mission state, or (None, None) when it says nothing new.

    stage "summon": enroute -> arrived.   stage "guide": guiding -> done.
    Either stage: help (rider must do something), waiting, lost, aborted.
    """
    if state in ("routing", "driving"):
        return ("enroute" if stage == "summon" else "guiding"), None
    if state == "waiting_for_help":
        return "help", PROMPTS.get(detail) or Prompt(detail or "help", "The car needs help", (detail or "").replace("_", " "), "Done")
    if state == "waiting_for_student":
        return "waiting", PROMPTS.get(detail, WAITING)
    if state == "lost":
        return "lost", LOST
    if state == "arrived":
        return ("arrived" if stage == "summon" else "done"), None
    if state == "aborted":
        return "aborted", None
    return None, None
