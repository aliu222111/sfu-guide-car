"""The voice agent's brain (spec section 9): transcript in, tool calls and a spoken reply out.

Claude Haiku 4.5 with exactly three tools: set_destination(room),
pause_or_resume(), confirm(). No free conversation beyond clarifying the room.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

MODEL = "claude-haiku-4-5"

SYSTEM = """You are the voice of a small guide robot at Simon Fraser University's Burnaby campus. \
It leads students to classrooms in the Academic Quadrangle (AQ), West Mall Centre (WMC) and \
Maggie Benston Centre (MBC). A student is speaking to you through the robot's microphone; \
your reply is read aloud by the robot's speaker.

Use the tools to act:
- set_destination when the student names a room. Room names look like "AQ 3150" or "WMC 2260". \
Speech recognition often garbles them ("A Q thirty-one fifty", "WMC twenty-two sixty"); turn them \
into that form. If the building is missing, ask which building.
- pause_or_resume when the student asks you to stop, wait, hold on, or carry on.
- confirm when the student says yes, they pressed the button, the doors are open, they are holding \
the door, or they are still with you.

Only help with getting to a room. For anything else, say briefly that you can only guide to \
rooms in AQ, WMC and MBC. Replies are spoken, so keep them to one short sentence with no lists, \
symbols or markdown."""

TOOLS = [
    {
        "name": "set_destination",
        "description": "Start guiding the student to a room. Call this once the room is clear.",
        "input_schema": {
            "type": "object",
            "properties": {"room": {"type": "string", "description": 'Building abbreviation and room number, e.g. "AQ 3150".'}},
            "required": ["room"],
            "additionalProperties": False,
        },
    },
    {
        "name": "pause_or_resume",
        "description": "Pause the trip if moving, or resume it if paused.",
        "input_schema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
    {
        "name": "confirm",
        "description": "The student confirms: yes, button pressed, doors open, holding the door, or still here.",
        "input_schema": {"type": "object", "properties": {}, "additionalProperties": False},
    },
]


@dataclass
class Actions:
    """What the agent can make the car do. Each returns a short result for the model."""

    set_destination: Callable[[str], str]
    pause_or_resume: Callable[[], str]
    confirm: Callable[[], str]


@dataclass
class VoiceAgent:
    client: object
    actions: Actions
    model: str = MODEL
    max_turns: int = 4  # tool rounds per utterance
    idle_reset: float = 120.0  # seconds: a new student, a new conversation
    history_limit: int = 24
    messages: list = field(default_factory=list)
    last_time: float = 0.0

    def _run_tool(self, name: str, args: dict) -> tuple[str, bool]:
        try:
            if name == "set_destination":
                room = args.get("room")
                if not isinstance(room, str) or not room.strip():
                    return "Error: room is required.", True
                return self.actions.set_destination(room.strip()), False
            if name == "pause_or_resume":
                return self.actions.pause_or_resume(), False
            if name == "confirm":
                return self.actions.confirm(), False
        except Exception as e:  # report to the model rather than crash the node
            return f"Error: {e}", True
        return f"Error: unknown tool {name}.", True

    def _trim(self):
        # Keep whole exchanges: cut only at a user message that is plain text.
        while len(self.messages) > self.history_limit:
            cut = next(
                (i for i in range(1, len(self.messages)) if self.messages[i]["role"] == "user" and isinstance(self.messages[i]["content"], str)),
                None,
            )
            if cut is None:
                self.messages = self.messages[-1:]
                return
            self.messages = self.messages[cut:]

    def handle(self, transcript: str, context: str = "", now: float | None = None) -> str:
        """Process one utterance; returns the text to speak."""
        now = time.monotonic() if now is None else now
        if now - self.last_time > self.idle_reset:
            self.messages = []
        self.last_time = now
        text = transcript.strip()
        if not text:
            return ""
        if context:
            text = f"[Robot status: {context}]\n{text}"
        self.messages.append({"role": "user", "content": text})
        reply = ""
        for _ in range(self.max_turns):
            response = self.client.messages.create(
                model=self.model,
                max_tokens=512,
                system=SYSTEM,
                tools=TOOLS,
                messages=self.messages,
            )
            if response.stop_reason != "tool_use":
                # A reply cut off by max_tokens (or a refusal) can end in a
                # tool_use with no result; keeping it would make every later
                # request fail, so only text is kept.
                content = [b for b in response.content if b.type == "text"]
                if content:
                    self.messages.append({"role": "assistant", "content": content})
                else:
                    self.messages.pop()  # drop the unanswered user turn too
                reply = " ".join(b.text for b in content).strip()
                break
            self.messages.append({"role": "assistant", "content": response.content})
            reply = " ".join(b.text for b in response.content if b.type == "text").strip()
            results = []
            for block in response.content:
                if block.type == "tool_use":
                    out, is_err = self._run_tool(block.name, dict(block.input or {}))
                    results.append({"type": "tool_result", "tool_use_id": block.id, "content": out, "is_error": is_err})
            self.messages.append({"role": "user", "content": results})
        else:
            reply = reply or "Sorry, could you say that again?"
        self._trim()
        return reply or "Sorry, I can only help you get to a room in AQ, WMC or MBC."
