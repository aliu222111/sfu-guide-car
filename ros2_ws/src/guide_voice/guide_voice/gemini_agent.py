"""The voice agent on Google Gemini instead of Claude (guide.yaml ``llm: gemini``).

Same system prompt, same three tools, same behaviour as ``agent.VoiceAgent``:
transcript in, tool calls and one short spoken sentence out. Key in
GEMINI_API_KEY (use a paid key: the free tier lets Google use the
conversation). Automatic function calling is off so the car's tools run here,
where their results are known, and are reported back explicitly.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .agent import SYSTEM, TOOLS, Actions, run_tool

MODEL = "gemini-2.5-flash"  # pinned: a "latest" alias can move to a model that rejects thinking_budget  # pin a versioned model once one is chosen for the car


def _declarations(types):
    return [types.FunctionDeclaration(name=t["name"], description=t["description"],
                                      parameters_json_schema=t["input_schema"]) for t in TOOLS]


@dataclass
class GeminiVoiceAgent:
    client: object
    actions: Actions
    model: str = MODEL
    max_turns: int = 4  # tool rounds per utterance
    idle_reset: float = 120.0  # seconds: a new student, a new conversation
    history_limit: int = 24
    contents: list = field(default_factory=list)
    last_time: float = 0.0

    def _config(self):
        from google.genai import types

        return types.GenerateContentConfig(
            system_instruction=SYSTEM,
            tools=[types.Tool(function_declarations=_declarations(types))],
            automatic_function_calling=types.AutomaticFunctionCallingConfig(disable=True),
            max_output_tokens=512,
            thinking_config=types.ThinkingConfig(thinking_budget=0),  # replies are one sentence: answer fast
        )

    def _trim(self):
        # Keep whole exchanges: cut only at a user turn of plain text (not a tool result).
        def plain_user(c):
            return c.role == "user" and all(getattr(p, "function_response", None) is None for p in c.parts or [])

        while len(self.contents) > self.history_limit:
            cut = next((i for i in range(1, len(self.contents)) if plain_user(self.contents[i])), None)
            if cut is None:
                self.contents = self.contents[-1:]
                return
            self.contents = self.contents[cut:]

    def handle(self, transcript: str, context: str = "", now: float | None = None) -> str:
        """Process one utterance; returns the text to speak."""
        from google.genai import types

        now = time.monotonic() if now is None else now
        if now - self.last_time > self.idle_reset:
            self.contents = []
        self.last_time = now
        text = transcript.strip()
        if not text:
            return ""
        if context:
            text = f"[Robot status: {context}]\n{text}"
        self.contents.append(types.Content(role="user", parts=[types.Part.from_text(text=text)]))
        reply = ""
        config = self._config()
        for _ in range(self.max_turns):
            response = self.client.models.generate_content(model=self.model, contents=self.contents, config=config)
            calls = response.function_calls or []
            candidate = response.candidates[0].content if response.candidates else None
            if not calls:
                reply = (response.text or "").strip() if candidate is not None else ""
                if candidate is not None and reply:
                    self.contents.append(candidate)
                else:
                    self.contents.pop()  # blocked or empty: drop the unanswered user turn
                break
            self.contents.append(candidate)
            parts = []
            for call in calls:
                out, is_err = run_tool(self.actions, call.name, dict(call.args or {}))
                parts.append(types.Part.from_function_response(name=call.name, response={"error" if is_err else "result": out}))
            self.contents.append(types.Content(role="user", parts=parts))
        else:
            reply = reply or "Sorry, could you say that again?"
            # Close the exchange with a model turn so the next utterance does not
            # follow a tool result as a second user turn in a row.
            self.contents.append(types.Content(role="model", parts=[types.Part.from_text(text=reply)]))
        self._trim()
        return reply or "Sorry, I can only help you get to a room in AQ, WMC or MBC."
