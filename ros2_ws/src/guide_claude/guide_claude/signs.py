"""Room-sign reading on the Claude subscription (spec section 16).

Each frame is one user turn in a persistent session. Images make the context
grow quickly, so the session is restarted every ``frames_per_session`` frames
(a spare one is warmed up in the background so reads keep flowing). Mind the
subscription's usage limits: the default rate on the car is one frame every
5 s (guide.yaml), not the 1 to 2 Hz of the local OCR.
"""

from __future__ import annotations

import base64

from .session import AgentSession, user_message


class SubscriptionSignReader:
    def __init__(self, prompt: str, model: str = "claude-haiku-4-5", frames_per_session: int = 20, client_factory=None):
        self.prompt = prompt
        self.frames_per_session = frames_per_session

        def options():
            from claude_agent_sdk import ClaudeAgentOptions

            return ClaudeAgentOptions(
                model=model,
                system_prompt="You read room numbers from camera frames and answer with JSON only.",
                tools=[],
                allowed_tools=[],
                permission_mode="dontAsk",
                setting_sources=[],
                max_turns=1,
            )

        self._make = lambda: AgentSession(options, client_factory)
        self.session = self._make()
        self.session.prewarm()
        self.spare = None

    def read_text(self, jpeg: bytes, timeout: float = 20.0) -> str:
        """The model's reply for one JPEG frame ("" on error)."""
        if self.session.turns >= self.frames_per_session - 2 and self.spare is None:
            self.spare = self._make()
            self.spare.prewarm()
        if self.session.turns >= self.frames_per_session and self.spare is not None:
            old, self.session, self.spare = self.session, self.spare, None
            old.close()
        data = base64.standard_b64encode(jpeg).decode("ascii")
        msg = user_message([
            {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": data}},
            {"type": "text", "text": self.prompt},
        ])
        reply = self.session.ask(msg, timeout=timeout)
        return "" if reply.is_error else reply.text
