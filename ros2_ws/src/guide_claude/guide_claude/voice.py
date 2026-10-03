"""The voice agent's brain on the Claude subscription (spec section 9).

Same contract as ``guide_voice.agent.VoiceAgent``: ``handle(transcript,
context, now)`` returns the text to speak, and the model may call exactly
three tools, run in-process as an SDK MCP server.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from .session import AgentSession, Reply

SERVER = "car"
TOOL_NAMES = ("set_destination", "pause_or_resume", "confirm")


def make_tools(set_destination: Callable[[str], str], pause_or_resume: Callable[[], str], confirm: Callable[[], str]):
    from claude_agent_sdk import tool

    def result(text: str, error: bool = False) -> dict[str, Any]:
        out: dict[str, Any] = {"content": [{"type": "text", "text": text}]}
        if error:
            out["is_error"] = True
        return out

    @tool("set_destination", "Start guiding the student to a room. Call this once the room is clear.",
          {"type": "object", "properties": {"room": {"type": "string", "description": 'Building and room, e.g. "AQ 3150".'}},
           "required": ["room"]})
    async def t_set(args):
        room = args.get("room")
        if not isinstance(room, str) or not room.strip():
            return result("Error: room is required.", True)
        try:
            return result(set_destination(room.strip()))
        except Exception as e:
            return result(f"Error: {e}", True)

    @tool("pause_or_resume", "Pause the trip if moving, or resume it if paused.", {"type": "object", "properties": {}})
    async def t_pause(args):
        try:
            return result(pause_or_resume())
        except Exception as e:
            return result(f"Error: {e}", True)

    @tool("confirm", "The student confirms: yes, button pressed, doors open, holding the door, or still here.",
          {"type": "object", "properties": {}})
    async def t_confirm(args):
        try:
            return result(confirm())
        except Exception as e:
            return result(f"Error: {e}", True)

    return [t_set, t_pause, t_confirm]


class SubscriptionVoiceAgent:
    def __init__(self, actions, system_prompt: str, model: str = "claude-haiku-4-5", idle_reset: float = 120.0,
                 max_turns: int = 4, client_factory=None):
        self.actions = actions
        self.idle_reset = idle_reset
        self.last_time = 0.0

        def options():
            from claude_agent_sdk import ClaudeAgentOptions, create_sdk_mcp_server

            server = create_sdk_mcp_server(SERVER, tools=make_tools(actions.set_destination, actions.pause_or_resume, actions.confirm))
            return ClaudeAgentOptions(
                model=model,
                system_prompt=system_prompt,
                tools=[],  # no built-in Claude Code tools: no shell, no files
                mcp_servers={SERVER: server},
                allowed_tools=[f"mcp__{SERVER}__{n}" for n in TOOL_NAMES],
                permission_mode="dontAsk",  # anything not allowed above is refused
                setting_sources=[],  # ignore the owner's personal Claude Code settings
                max_turns=max_turns,
            )

        self.session = AgentSession(options, client_factory)
        self.session.prewarm()

    def handle(self, transcript: str, context: str = "", now: float | None = None) -> str:
        now = time.monotonic() if now is None else now
        if self.last_time and now - self.last_time > self.idle_reset:
            self.session.reset()  # a new student, a new conversation
            self.session.prewarm()
        self.last_time = now
        text = transcript.strip()
        if not text:
            return ""
        if context:
            text = f"[Robot status: {context}]\n{text}"
        reply: Reply = self.session.ask(text, timeout=30.0)
        if reply.is_error:
            return "Sorry, could you say that again?"
        return reply.text or "Sorry, I can only help you get to a room in AQ, WMC or MBC."
