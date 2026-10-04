"""The car's speaker: Piper in-process, playback through sounddevice, push-to-talk cuts it off."""

import sys
import time
import types
import wave

import numpy as np

from guide_voice.speech import Speaker, audio_device, read_wav


class FakeVoice:
    def synthesize_wav(self, text, w):
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(22050)
        w.writeframes((np.ones(2205, np.int16) * 1000).tobytes())  # 0.1 s


def fake_sounddevice(monkeypatch):
    sd = types.SimpleNamespace(played=[], stopped=0)
    sd.play = lambda data, rate, device=None: sd.played.append((data.shape, rate, device))
    sd.wait = lambda: None
    sd.stop = lambda: setattr(sd, "stopped", sd.stopped + 1)
    monkeypatch.setitem(sys.modules, "sounddevice", sd)
    return sd


def test_speaks_through_sounddevice(monkeypatch):
    sd = fake_sounddevice(monkeypatch)
    sp = Speaker("voice.onnx", device=3)
    sp._voice = FakeVoice()
    sp.say("Turn left.")
    deadline = time.monotonic() + 2
    while sp.spoken < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sp.spoken == 1 and sd.played == [((2205, 1), 22050, 3)]


def test_push_to_talk_stops_and_holds(monkeypatch):
    sd = fake_sounddevice(monkeypatch)
    sp = Speaker("voice.onnx")
    sp._voice = FakeVoice()
    sp.hold(True)
    assert sd.stopped == 1
    sp.say("Queued while the student talks.")
    time.sleep(0.2)
    assert sp.spoken == 0  # held
    sp.hold(False)
    deadline = time.monotonic() + 2
    while sp.spoken < 1 and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sp.spoken == 1


def test_missing_voice_is_reported_not_raised(monkeypatch):
    fake_sounddevice(monkeypatch)
    sp = Speaker("/nonexistent/voice.onnx")
    sp.say("Hello.")
    deadline = time.monotonic() + 5
    while not sp.error and time.monotonic() < deadline:
        time.sleep(0.01)
    assert sp.error and sp.spoken == 0


def test_read_wav_and_devices(tmp_path):
    p = tmp_path / "a.wav"
    with wave.open(str(p), "wb") as w:
        FakeVoice().synthesize_wav("", w)
    data, rate = read_wav(p)
    assert data.shape == (2205, 1) and rate == 22050
    assert audio_device("") is None and audio_device("3") == 3 and audio_device("USB Audio") == "USB Audio"


def test_voice_check_runs_without_audio_or_key(monkeypatch, capsys, tmp_path):
    from guide_voice import voice_check

    fake_sounddevice(monkeypatch)
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.delenv("GOOGLE_API_KEY", raising=False)
    assert voice_check.main(["--text", "take me to AQ 3046", "--voice", str(tmp_path / "none.onnx")]) == 0
    out = capsys.readouterr().out
    assert "No Piper voice" in out and "speaker failed" in out and "No GEMINI_API_KEY" in out
