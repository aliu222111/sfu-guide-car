"""Audio in and out: push-to-talk recording, faster-whisper, Piper (spec section 9)."""

from __future__ import annotations

import queue
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import numpy as np

SAMPLE_RATE = 16000


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


class Transcriber:
    """faster-whisper; ``small`` on a GPU, ``base`` int8 on CPU (spec section 15)."""

    HINT = "AQ, WMC, MBC, Academic Quadrangle, West Mall Centre, Maggie Benston Centre, room 3150."

    def __init__(self, model: str = "base", device: str = "auto", compute_type: str = "int8"):
        from faster_whisper import WhisperModel

        self.model = WhisperModel(model, device=device, compute_type=compute_type)

    def __call__(self, audio: np.ndarray) -> str:
        if audio.size < SAMPLE_RATE * 0.3:
            return ""
        segments, _ = self.model.transcribe(audio, language="en", beam_size=3, initial_prompt=self.HINT, vad_filter=True)
        return " ".join(s.text.strip() for s in segments).strip()


class Speaker:
    """Speaks queued text with Piper on the car's speaker, one message at a time.

    ``hold(True)`` stops the current sentence and keeps queued ones until
    ``hold(False)``, so the car does not talk over (or into the recording of)
    a student holding the push-to-talk button.
    """

    def __init__(self, voice_model: str, piper_bin: str = "piper", player: str = "aplay"):
        self.voice_model = voice_model
        self.piper_bin = piper_bin
        self.player = player
        self.q: queue.Queue[str] = queue.Queue()
        self._held = threading.Event()
        self._playing: subprocess.Popen | None = None
        threading.Thread(target=self._loop, daemon=True).start()

    def say(self, text: str):
        if text.strip():
            self.q.put(text.strip())

    def hold(self, on: bool):
        if on:
            self._held.set()
            p = self._playing
            if p is not None and p.poll() is None:
                p.terminate()
        else:
            self._held.clear()

    def _loop(self):
        while True:
            text = self.q.get()
            while self._held.is_set():
                time.sleep(0.05)
            try:
                with tempfile.TemporaryDirectory() as d:
                    wav = Path(d) / "say.wav"
                    subprocess.run([self.piper_bin, "--model", self.voice_model, "--output_file", str(wav)], input=text.encode(), check=True, capture_output=True, timeout=30)
                    if self._held.is_set():
                        self.q.put(text)  # interrupted before playing: say it after
                        continue
                    self._playing = subprocess.Popen([self.player, "-q", str(wav)])
                    self._playing.wait(timeout=60)
            except (OSError, subprocess.SubprocessError):
                pass  # no speaker is not a reason to crash; the phone also speaks
            finally:
                self._playing = None
