"""ElevenLabs voice and transcription, and the Gemini agent, against fake clients."""

import io
import wave
from types import SimpleNamespace

import numpy as np
import pytest

from guide_voice.agent import Actions
from guide_voice.cloud import ElevenLabsSpeaker, ElevenLabsTranscriber, Unheard, to_wav_bytes


def _wav(n=100):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1), w.setsampwidth(2), w.setframerate(22050), w.writeframes(b"\0\0" * n)
    return buf.getvalue()


class FakeTTS:
    def __init__(self, fail=False):
        self.calls, self.fail = [], fail

    def convert(self, voice_id, **kw):
        self.calls.append((voice_id, kw))
        if self.fail:
            raise ConnectionError("no network")
        pcm = b"\x00\x10" * 2205  # 0.1 s of 16-bit PCM, streamed in chunks
        return iter([pcm[:1000], pcm[1000:]])


def test_elevenlabs_voice_cached_and_piper_fallback(tmp_path, monkeypatch):
    piper = []
    monkeypatch.setattr("guide_voice.speech.Speaker.synthesize", lambda self, text, wav: piper.append(text))
    tts = FakeTTS()
    sp = ElevenLabsSpeaker("voice.onnx", client=SimpleNamespace(text_to_speech=tts))
    out = tmp_path / "a.wav"
    sp.synthesize("In 5 metres, turn right.", out)
    sp.synthesize("In 5 metres, turn right.", out)  # repeated announcement: from the cache
    assert out.read_bytes().startswith(b"RIFF") and len(tts.calls) == 1
    assert tts.calls[0][1]["model_id"] == "eleven_flash_v2_5" and tts.calls[0][1]["output_format"] == "pcm_24000"
    tts.fail = True
    sp.synthesize("We've arrived.", tmp_path / "b.wav")  # WiFi gone: Piper speaks
    assert piper == ["We've arrived."]


def test_room_codes_read_as_people_say_them():
    from guide_voice.cloud import for_speech

    assert for_speech("AQ 3128, on the 3000 level") == "A Q 31 28, on the 3000 level"
    assert for_speech("ASB 9815 and AQ 3005") == "A S B 98 15 and A Q 30 oh 5"
    assert for_speech("room 101") == "room 101"


def test_wav_bytes_round_trip():
    audio = np.sin(np.linspace(0, 100, 16000)).astype(np.float32) * 0.5
    with wave.open(io.BytesIO(to_wav_bytes(audio))) as w:
        assert (w.getframerate(), w.getnchannels(), w.getsampwidth(), w.getnframes()) == (16000, 1, 2, 16000)


def test_elevenlabs_transcriber():
    calls = []

    class STT:
        fail = False

        def convert(self, **kw):
            calls.append(kw)
            if STT.fail:
                raise TimeoutError("slow WiFi")
            return SimpleNamespace(text=" AQ 3150 please ")

    t = ElevenLabsTranscriber(client=SimpleNamespace(speech_to_text=STT()))
    audio = np.zeros(16000, np.float32)
    assert t(audio, extra_terms=("AQ 3046",)) == "AQ 3150 please"
    name, data, mime = calls[0]["file"]
    assert data.startswith(b"RIFF") and mime == "audio/wav"
    assert "AQ" in calls[0]["keyterms"] and "AQ 3046" in calls[0]["keyterms"]
    STT.fail = True
    with pytest.raises(Unheard):  # no network: the node says it cannot hear just now
        t(audio)
    assert t(np.zeros(100, np.float32)) == ""  # too short to be speech: no call


genai_types = pytest.importorskip("google.genai.types")


def _response(text=None, calls=()):
    parts = [genai_types.Part(function_call=genai_types.FunctionCall(name=n, args=a)) for n, a in calls]
    if text:
        parts.append(genai_types.Part.from_text(text=text))
    content = genai_types.Content(role="model", parts=parts)
    return genai_types.GenerateContentResponse(candidates=[genai_types.Candidate(content=content)])


class FakeModels:
    def __init__(self, replies):
        self.replies, self.requests = list(replies), []

    def generate_content(self, model, contents, config):
        self.requests.append((model, list(contents), config))
        return self.replies.pop(0)


def test_gemini_agent_calls_tools_and_answers():
    from guide_voice.gemini_agent import GeminiVoiceAgent

    dest = []
    actions = Actions(lambda room: dest.append(room) or f"Guiding to {room}.", lambda: "Paused.", lambda: "Confirmed.")
    models = FakeModels([_response(calls=[("set_destination", {"room": " AQ 3150 "})]), _response(text="Follow me to AQ 3150.")])
    agent = GeminiVoiceAgent(SimpleNamespace(models=models), actions)
    reply = agent.handle("take me to a q thirty one fifty", context="idle", now=0.0)
    assert reply == "Follow me to AQ 3150." and dest == ["AQ 3150"]
    _, contents, config = models.requests[1]
    assert contents[-1].parts[0].function_response.response == {"result": "Guiding to AQ 3150."}
    assert config.automatic_function_calling.disable is True
    assert {f.name for f in config.tools[0].function_declarations} == {"set_destination", "pause_or_resume", "confirm"}
    assert [c.role for c in agent.contents] == ["user", "model", "user", "model"]


def test_gemini_agent_bad_tool_and_blocked_reply():
    from guide_voice.gemini_agent import GeminiVoiceAgent

    actions = Actions(lambda room: "x", lambda: "Paused.", lambda: "Confirmed.")
    models = FakeModels([_response(calls=[("set_destination", {})]), _response(text="Which room?"), _response()])
    agent = GeminiVoiceAgent(SimpleNamespace(models=models), actions)
    assert agent.handle("take me there", now=0.0) == "Which room?"
    assert models.requests[1][1][-1].parts[0].function_response.response == {"error": "Error: room is required."}
    n = len(agent.contents)
    assert agent.handle("something blocked", now=1.0)  # empty reply: a polite fallback, history unchanged
    assert len(agent.contents) == n
