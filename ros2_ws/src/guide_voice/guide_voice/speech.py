"""Audio in and out: push-to-talk recording and Piper, the offline voice (spec section 9).

Cloud alternatives (ElevenLabs) are in ``cloud.py`` and fall back to these.
"""

from __future__ import annotations

import queue
import tempfile
import threading
import time
import wave
from pathlib import Path

import numpy as np

SAMPLE_RATE = 16000


def audio_device(name: str):
    """A sounddevice device from a parameter: "" is the system default, digits an index."""
    name = (name or "").strip()
    return None if not name else int(name) if name.isdigit() else name


class Recorder:
    """Records from the default microphone while ``start()``..``stop()``."""

    def __init__(self, device=None, max_seconds: float = 15.0):
        self.device = device
        self.max_frames = int(max_seconds * SAMPLE_RATE)
        self._chunks: list[np.ndarray] = []
        self._stream = None
        self._lock = threading.Lock()

    def start(self):
        import sounddevice as sd

        with self._lock:
            if self._stream is not None:
                return
            self._chunks = []

            def cb(indata, frames, time_info, status):
                if sum(len(c) for c in self._chunks) < self.max_frames:
                    self._chunks.append(indata[:, 0].copy())

            self._stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="float32", device=self.device, callback=cb)
            self._stream.start()

    def stop(self) -> np.ndarray:
        with self._lock:
            if self._stream is None:
                return np.zeros(0, np.float32)
            self._stream.stop()
            self._stream.close()
            self._stream = None
            return np.concatenate(self._chunks) if self._chunks else np.zeros(0, np.float32)


def trim_silence(audio: np.ndarray, threshold: float = 0.01, pad: float = 0.2) -> np.ndarray:
    """Cut leading and trailing quiet; empty if the clip never gets loud."""
    if audio.size == 0:
        return audio
    win = int(0.02 * SAMPLE_RATE)
    n = audio.size // win
    if n == 0:
        return audio[:0]
    rms = np.sqrt((audio[: n * win].reshape(n, win) ** 2).mean(axis=1))
    loud = np.nonzero(rms > threshold)[0]
    if loud.size == 0:
        return audio[:0]
    p = int(pad * SAMPLE_RATE)
    return audio[max(0, loud[0] * win - p) : min(audio.size, (loud[-1] + 1) * win + p)]


def read_wav(path: Path) -> tuple[np.ndarray, int]:
    """16-bit PCM samples (frames, channels) and the sample rate."""
    with wave.open(str(path), "rb") as w:
        data = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
        return data.reshape(-1, w.getnchannels()), w.getframerate()


class Speaker:
    """Speaks queued text with Piper on the car's speaker, one message at a time.

    Piper runs in-process (the ``piper-tts`` package; the voice loads once) and
    plays through sounddevice, the same audio library the microphone uses.
    ``hold(True)`` stops the current sentence and keeps queued ones until
    ``hold(False)``, so the car does not talk over (or into the recording of)
    a student holding the push-to-talk button.
    """

    def __init__(self, voice_model: str, device=None):
        self.voice_model = voice_model
        self.device = device
        self._voice = None
        self.q: queue.Queue[str] = queue.Queue()
        self._held = threading.Event()
        self.spoken = 0
        self.error = ""  # the last failure, for voice_check and the logs
        threading.Thread(target=self._loop, daemon=True).start()

    def say(self, text: str):
        if text.strip():
            self.q.put(text.strip())

    def hold(self, on: bool):
        if on:
            self._held.set()
            try:
                import sounddevice as sd

                sd.stop()
            except Exception:
                pass
        else:
            self._held.clear()

    def synthesize(self, text: str, wav: Path):
        """Write ``text`` as speech to ``wav``. Piper here; cloud voices override it."""
        if self._voice is None:
            from piper import PiperVoice

            self._voice = PiperVoice.load(self.voice_model)
        with wave.open(str(wav), "wb") as w:
            self._voice.synthesize_wav(text, w)

    def play(self, wav: Path):
        import sounddevice as sd

        data, rate = read_wav(wav)
        sd.play(data, rate, device=self.device)
        sd.wait()

    def _loop(self):
        while True:
            text = self.q.get()
            while self._held.is_set():
                time.sleep(0.05)
            try:
                with tempfile.TemporaryDirectory() as d:
                    wav = Path(d) / "say.wav"
                    self.synthesize(text, wav)
                    if self._held.is_set():
                        self.q.put(text)  # interrupted before playing: say it after
                        continue
                    self.play(wav)
                    self.spoken += 1
            except Exception as e:  # no speaker or no voice file is not a reason to crash; the phone also speaks
                self.error = f"{type(e).__name__}: {e}"
