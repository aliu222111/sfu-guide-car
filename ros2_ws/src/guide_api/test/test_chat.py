"""Chat with ElevenLabs: the concierge with stand-in services, the request shapes, and over HTTP."""

import base64
import json
import urllib.error
import urllib.request

from guide_api.chat import ChatError, Concierge, ElevenLabsScribe, ElevenLabsVoice, GeminiBrain, building_facts
from guide_api.http_api import serve
from guide_api.service import CarLink, ScoutService

ROOMS = {
    "ASB 9896": {"type": "Seminar", "floor": "ASB-09", "floor_name": "9000 Level"},
    "ASB 990": {"type": "washroom", "floor": "ASB-09", "floor_name": "9000 Level"},
    "ASB 9700": {"type": "lab", "floor": "ASB-09", "floor_name": "9000 Level"},
}


def test_building_facts_lists_washrooms_and_labs():
    assert building_facts(ROOMS) == "9000 Level: 3 rooms; washrooms: ASB 990; computer labs: ASB 9700"


def test_concierge_keeps_the_conversation_and_speaks():
    seen = []

    def brain(system, history, run_tool):
        seen.append((system, [h["text"] for h in history]))
        return "Try the noodle place by the bus loop. I can't check if it is open."

    c = Concierge(brain, ROOMS, speak=lambda text: b"MP3" + text.encode()[:4])
    a = c.ask("s1", "  any good   Chinese food? ", "the car is busy")
    assert a["heard"] == "any good Chinese food?" and a["room"] is None
    assert base64.b64decode(a["audio"]).startswith(b"MP3") and a["audioType"] == "audio/mpeg"
    assert "Right now: the car is busy" in seen[0][0] and "washrooms: ASB 990" in seen[0][0]
    c.ask("s1", "and coffee?")
    assert seen[1][1] == ["any good Chinese food?", "Try the noodle place by the bus loop. I can't check if it is open.", "and coffee?"]
    c.ask("s2", "hello")
    assert seen[2][1] == ["hello"]                       # another rider: another conversation


def test_show_room_offers_a_real_room_and_refuses_an_unknown_one():
    def brain(system, history, run_tool):
        return run_tool("show_room", {"room": "asb9896"}) + " | " + run_tool("show_room", {"room": "ASB 9897"})

    a = Concierge(brain, ROOMS).ask("s", "take me to 9896")
    assert a["room"] == {"key": "ASB 9896", "type": "Seminar", "floor": "ASB-09", "floorName": "9000 Level"}
    assert "ASB 9896 is on the map" in a["reply"] and "ASB 9897 is not on the map. Closest: ASB 9896" in a["reply"]
    assert a["audio"] is None                              # no voice configured: text only


def test_empty_question_and_a_voice_failure_do_not_break_the_reply():
    def broken(text):
        raise ChatError("no credits")

    c = Concierge(lambda s, h, t: "Hello.", ROOMS, speak=broken)
    assert c.ask("s", "   ")["reply"].startswith("I didn't catch that")
    a = c.ask("s", "hi")
    assert a["reply"] == "Hello." and a["audio"] is None


def test_gemini_request_and_tool_round():
    sent = []
    answers = [
        {"candidates": [{"content": {"role": "model", "parts": [{"functionCall": {"name": "show_room", "args": {"room": "ASB 990"}}}]}}]},
        {"candidates": [{"content": {"role": "model", "parts": [{"text": "The nearest washroom is ASB 990."}]}}]},
    ]

    def post(url, headers, body, timeout):
        sent.append((url, headers, json.loads(body)))
        return json.dumps(answers[len(sent) - 1]).encode()

    brain = GeminiBrain("KEY", post=post)
    out = brain("SYS", [{"role": "user", "text": "washroom?"}], lambda name, args: f"{name}:{args['room']}")
    assert out == "The nearest washroom is ASB 990."
    url, headers, body = sent[0]
    assert url.endswith("/models/gemini-2.5-flash:generateContent") and headers["x-goog-api-key"] == "KEY"
    assert body["systemInstruction"]["parts"][0]["text"] == "SYS"
    assert body["tools"][0]["functionDeclarations"][0]["name"] == "show_room"
    second = sent[1][2]["contents"]
    assert second[1]["role"] == "model" and "functionCall" in second[1]["parts"][0]
    assert second[2]["parts"][0]["functionResponse"] == {"name": "show_room", "response": {"result": "show_room:ASB 990"}}


def test_elevenlabs_requests():
    sent = []

    def post(url, headers, body, timeout):
        sent.append((url, headers, body))
        return b'{"text": " hello there "}' if "speech-to-text" in url else b"ID3audio"

    assert ElevenLabsVoice("K", post=post)("Hi") == b"ID3audio"
    url, headers, body = sent[0]
    assert "/text-to-speech/21m00Tcm4TlvDq8ikWAM?output_format=mp3_44100_128" in url and headers["xi-api-key"] == "K"
    assert json.loads(body) == {"text": "Hi", "model_id": "eleven_flash_v2_5"}

    assert ElevenLabsScribe("K", post=post)(b"\x1aE\xdf\xa3webm", "audio/webm;codecs=opus") == "hello there"
    url, headers, body = sent[1]
    assert url.endswith("/speech-to-text") and headers["Content-Type"].startswith("multipart/form-data; boundary=")
    assert b'name="model_id"\r\n\r\nscribe_v1' in body and b'filename="speech.webm"' in body and b"\x1aE\xdf\xa3webm" in body


def test_from_env_needs_a_gemini_key():
    assert Concierge.from_env(ROOMS, env={}) is None
    c = Concierge.from_env(ROOMS, env={"GEMINI_API_KEY": "g"})
    assert c.status() == {"enabled": True, "canHear": False, "canSpeak": False}
    c = Concierge.from_env(ROOMS, env={"GEMINI_API_KEY": "g", "ELEVENLABS_API_KEY": "e"})
    assert c.status() == {"enabled": True, "canHear": True, "canSpeak": True}


class _Link(CarLink):
    def send_heartbeat(self): pass


def _call(base, method, path, body=None, raw=None, ctype="application/json"):
    data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(base + path, method=method, data=data, headers={"Content-Type": ctype})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_chat_over_http():
    service = ScoutService(_Link(), ROOMS)
    heard = []
    chat = Concierge(lambda s, h, t: "You said: " + h[-1]["text"], ROOMS,
                     hear=lambda audio, mime: heard.append((audio, mime)) or "spoken words", speak=lambda text: b"MP3")
    server = serve(service, port=0, host="127.0.0.1", chat=chat)
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        assert _call(base, "GET", "/api/chat/status")[1] == {"enabled": True, "canHear": True, "canSpeak": True}
        code, a = _call(base, "POST", "/api/chat", {"session": "x", "text": "hi"})
        assert code == 200 and a["reply"] == "You said: hi" and a["audio"] == base64.b64encode(b"MP3").decode()
        code, a = _call(base, "POST", "/api/chat/audio?session=x", raw=b"RECORDING", ctype="audio/webm")
        assert code == 200 and a["heard"] == "spoken words" and heard == [(b"RECORDING", "audio/webm")]
        assert _call(base, "POST", "/api/chat/audio", raw=b"", ctype="audio/webm")[0] == 400
    finally:
        server.shutdown()

    off = serve(service, port=0, host="127.0.0.1")           # no keys: the app is told the chat is off
    base = f"http://127.0.0.1:{off.server_address[1]}"
    try:
        assert _call(base, "GET", "/api/chat/status")[1]["enabled"] is False
        assert _call(base, "POST", "/api/chat", {"text": "hi"})[0] == 503
    finally:
        off.shutdown()
