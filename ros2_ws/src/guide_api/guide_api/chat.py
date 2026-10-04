"""Chat with ElevenLabs: talk to the guide from the app, before the car arrives (no ROS here).

The rider speaks or types; ElevenLabs Scribe writes down what was said, Gemini
answers (it can look a room up and offer it as a destination), and an
ElevenLabs voice reads the answer back. The same services the car's own
microphone uses (guide_voice), reached here over their HTTP APIs with the
standard library, so the rider API still needs nothing installed.

Unlike the car's voice agent, which only talks about getting to a room, this
one also answers general questions (food, coffee, where things are).

Keys, in the environment of whatever runs the API:
  ELEVENLABS_API_KEY   hearing and voice     https://elevenlabs.io/app/settings/api-keys
  GEMINI_API_KEY       the answers           https://aistudio.google.com/apikey
"""

from __future__ import annotations

import base64
import difflib
import json
import os
import time
import urllib.error
import urllib.request
import uuid

from .service import normalise_key

ELEVEN = "https://api.elevenlabs.io/v1"
GEMINI = "https://generativelanguage.googleapis.com/v1beta/models"
TTS_MODEL = "eleven_flash_v2_5"          # as guide_voice/cloud.py
STT_MODEL = "scribe_v1"
VOICE_ID = "21m00Tcm4TlvDq8ikWAM"        # "Rachel", the car's voice
GEMINI_MODEL = "gemini-2.5-flash"

SYSTEM = """You are the voice of Sparky Scout, a small guide car at Simon Fraser University's Burnaby campus. \
A student is talking to you in the Sparky Scout app, often while the car is still on its way to them. \
Your reply is read aloud by an ElevenLabs voice and shown as a chat bubble.

You can:
- Answer questions about getting around the Applied Sciences Building (ASB): rooms, washrooms, \
computer labs, stairs and the elevator. The context lists what is on the map.
- Recommend food, coffee, study spots and other things on or near the Burnaby campus from what you \
know. You cannot check opening hours, prices or whether a place still exists: say so in a few words \
when it matters, and never invent a place you are not sure of.
- Call show_room when the student names a room or asks for a kind of place that is on the map \
(a washroom, a computer lab). The app then shows it with a button to go there. The car only drives \
inside ASB; for anywhere else, give directions in words.

Replies are spoken: two or three short sentences, plain words, no lists, symbols or markdown."""

TOOLS = [{
    "name": "show_room",
    "description": "Show a room on the app's map with a button to go there. Use the building and number, e.g. \"ASB 9896\".",
    "parameters": {"type": "object", "properties": {"room": {"type": "string", "description": "Building abbreviation and room number."}}, "required": ["room"]},
}]


class ChatError(Exception):
    """A service could not be reached, or refused (no network, bad key, no credits)."""


def post(url: str, headers: dict, body: bytes, timeout: float) -> bytes:
    req = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read()
    except urllib.error.HTTPError as e:
        raise ChatError(f"{e.code} {e.read()[:200].decode('utf-8', 'replace')}") from None
    except OSError as e:
        raise ChatError(str(e)) from None


class ElevenLabsVoice:
    """Text to an MP3 the browser can play."""

    def __init__(self, key: str, voice_id: str = VOICE_ID, model: str = TTS_MODEL, timeout: float = 15.0, post=post):
        self.key, self.voice_id, self.model, self.timeout, self.post = key, voice_id, model, timeout, post

    def __call__(self, text: str) -> bytes:
        body = json.dumps({"text": text, "model_id": self.model}).encode()
        return self.post(f"{ELEVEN}/text-to-speech/{self.voice_id}?output_format=mp3_44100_128",
                         {"xi-api-key": self.key, "Content-Type": "application/json", "Accept": "audio/mpeg"}, body, self.timeout)


class ElevenLabsScribe:
    """A recording from the browser (webm, mp4, wav ...) to text."""

    def __init__(self, key: str, model: str = STT_MODEL, timeout: float = 30.0, post=post):
        self.key, self.model, self.timeout, self.post = key, model, timeout, post

    def __call__(self, audio: bytes, mime: str = "audio/webm") -> str:
        boundary = uuid.uuid4().hex
        ext = {"audio/webm": "webm", "audio/mp4": "mp4", "audio/mpeg": "mp3", "audio/wav": "wav", "audio/ogg": "ogg"}.get(mime.split(";")[0].strip(), "webm")

        def field(name, value):
            return f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()

        body = (field("model_id", self.model) + field("language_code", "en") + field("tag_audio_events", "false")
                + f'--{boundary}\r\nContent-Disposition: form-data; name="file"; filename="speech.{ext}"\r\nContent-Type: {mime}\r\n\r\n'.encode()
                + audio + f"\r\n--{boundary}--\r\n".encode())
        out = self.post(f"{ELEVEN}/speech-to-text", {"xi-api-key": self.key, "Content-Type": f"multipart/form-data; boundary={boundary}"}, body, self.timeout)
        return (json.loads(out).get("text") or "").strip()


class GeminiBrain:
    """One question in, one spoken answer out; runs the tool calls in between."""

    def __init__(self, key: str, model: str = GEMINI_MODEL, timeout: float = 30.0, post=post, max_rounds: int = 3):
        self.key, self.model, self.timeout, self.post, self.max_rounds = key, model, timeout, post, max_rounds

    def __call__(self, system: str, history: list[dict], run_tool) -> str:
        contents = [{"role": h["role"], "parts": [{"text": h["text"]}]} for h in history]
        for _ in range(self.max_rounds):
            body = json.dumps({
                "systemInstruction": {"parts": [{"text": system}]},
                "contents": contents,
                "tools": [{"functionDeclarations": TOOLS}],
                "generationConfig": {"maxOutputTokens": 400, "thinkingConfig": {"thinkingBudget": 0}},
            }).encode()
            out = json.loads(self.post(f"{GEMINI}/{self.model}:generateContent", {"x-goog-api-key": self.key, "Content-Type": "application/json"}, body, self.timeout))
            parts = ((out.get("candidates") or [{}])[0].get("content") or {}).get("parts") or []
            calls = [p["functionCall"] for p in parts if "functionCall" in p]
            if not calls:
                return " ".join(p["text"] for p in parts if p.get("text")).strip()
            contents.append({"role": "model", "parts": parts})
            contents.append({"role": "user", "parts": [
                {"functionResponse": {"name": c.get("name", ""), "response": {"result": run_tool(c.get("name", ""), c.get("args") or {})}}} for c in calls]})
        return ""


def building_facts(rooms: dict) -> str:
    """What the map knows, in a few lines for the model: levels, washrooms, labs."""
    by_floor: dict[str, dict[str, list[str]]] = {}
    for key, r in rooms.items():
        kinds = by_floor.setdefault(r.get("floor_name") or r.get("floor", ""), {})
        t = (r.get("type") or "").lower()
        kind = "washrooms" if "wash" in t else "computer labs" if "lab" in t else None
        if kind:
            kinds.setdefault(kind, []).append(key)
        kinds.setdefault("count", []).append(key)
    lines = []
    for floor in sorted(by_floor):
        k = by_floor[floor]
        bits = [f"{len(k['count'])} rooms"] + [f"{name}: {', '.join(sorted(k[name]))}" for name in ("washrooms", "computer labs") if name in k]
        lines.append(f"{floor}: " + "; ".join(bits))
    return "\n".join(lines)


class Concierge:
    def __init__(self, brain, rooms: dict | None = None, *, hear=None, speak=None, clock=time.monotonic,
                 history_limit: int = 12, idle_reset: float = 600.0):
        self.brain, self.hear, self.speak = brain, hear, speak
        self.rooms = rooms or {}
        self.facts = building_facts(self.rooms)
        self.clock = clock
        self.history_limit, self.idle_reset = history_limit, idle_reset
        self.sessions: dict[str, dict] = {}

    @classmethod
    def from_env(cls, rooms: dict | None = None, env=os.environ) -> "Concierge | None":
        """The real thing, or None when no Gemini key is set (the app then hides the chat)."""
        g, e = env.get("GEMINI_API_KEY", ""), env.get("ELEVENLABS_API_KEY", "")
        if not g:
            return None
        return cls(GeminiBrain(g, env.get("SCOUT_GEMINI_MODEL", GEMINI_MODEL)), rooms,
                   hear=ElevenLabsScribe(e) if e else None,
                   speak=ElevenLabsVoice(e, env.get("SCOUT_VOICE_ID", VOICE_ID)) if e else None)

    def status(self) -> dict:
        return {"enabled": True, "canHear": self.hear is not None, "canSpeak": self.speak is not None}

    def _show_room(self, picked: list, args: dict) -> str:
        key = normalise_key(str(args.get("room", "")))
        r = self.rooms.get(key)
        if r:
            picked.append({"key": key, "type": r.get("type", ""), "floor": r.get("floor", ""), "floorName": r.get("floor_name", "")})
            return f"{key} is on the map ({r.get('type') or 'room'}, {r.get('floor_name') or r.get('floor', '')}). The app now shows it with a button to go there."
        close = difflib.get_close_matches(key, list(self.rooms), n=3, cutoff=0.75)
        return f"{key} is not on the map." + (f" Closest: {', '.join(close)}." if close else "")

    def ask(self, session: str, text: str, context: str = "") -> dict:
        """One typed or transcribed question -> ``{"heard", "reply", "audio", "audioType", "room"}``."""
        text = " ".join((text or "").split())[:600]
        if not text:
            return {"heard": "", "reply": "I didn't catch that. Could you say it again?", "audio": None, "audioType": None, "room": None}
        now = self.clock()
        s = self.sessions.setdefault(session or "default", {"at": now, "history": []})
        if now - s["at"] > self.idle_reset:
            s["history"] = []
        s["at"] = now
        picked: list[dict] = []
        system = SYSTEM + ("\n\nOn the map of ASB:\n" + self.facts if self.facts else "") + (f"\n\nRight now: {context}" if context else "")
        history = s["history"] + [{"role": "user", "text": text}]
        reply = self.brain(system, history, lambda name, args: self._show_room(picked, args) if name == "show_room" else f"Error: unknown tool {name}.")
        reply = reply or "Sorry, I couldn't answer that just now."
        s["history"] = (history + [{"role": "model", "text": reply}])[-self.history_limit:]
        audio = None
        if self.speak:
            try:
                audio = self.speak(reply)
            except ChatError:
                audio = None  # the bubble still shows; the app just stays quiet
        return {"heard": text, "reply": reply, "audio": base64.b64encode(audio).decode() if audio else None,
                "audioType": "audio/mpeg" if audio else None, "room": picked[-1] if picked else None}

    def ask_audio(self, session: str, audio: bytes, mime: str, context: str = "") -> dict:
        if self.hear is None:
            raise ChatError("no ElevenLabs key: the app can only take typed questions")
        return self.ask(session, self.hear(audio, mime or "audio/webm"), context)
