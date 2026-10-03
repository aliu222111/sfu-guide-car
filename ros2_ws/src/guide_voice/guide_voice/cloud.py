"""ElevenLabs voice and speech-to-text, with the local engines as fallback.

Optional (guide.yaml ``tts: elevenlabs``, ``stt: elevenlabs``; key in
ELEVENLABS_API_KEY). The car must keep talking when campus WiFi drops, so
every call has a short timeout and any failure falls back to Piper or
faster-whisper for that utterance. Mission announcements repeat ("In 5
metres, turn right."), so synthesized speech is cached by text: it saves
credits and the network round trip.

Privacy: with ``stt: elevenlabs`` the student's recorded voice leaves the
laptop; with ``tts: elevenlabs`` only the car's own sentences do.
"""

from __future__ import annotations

import io
import wave
from functools import lru_cache
from pathlib import Path

import numpy as np

from .speech import SAMPLE_RATE, Speaker

TTS_MODEL = "eleven_flash_v2_5"  # the low-latency model
STT_MODEL = "scribe_v1"
VOICE_ID = "21m00Tcm4TlvDq8ikWAM"  # "Rachel", a stock voice; any voice id from the ElevenLabs library works
ROOM_TERMS = ["AQ", "WMC", "MBC", "Academic Quadrangle", "West Mall Centre", "Maggie Benston Centre"]


def _client(timeout: float, client=None):
    if client is not None:
        return client
    from elevenlabs.client import ElevenLabs

    return ElevenLabs(timeout=timeout)  # reads ELEVENLABS_API_KEY


class ElevenLabsSpeaker(Speaker):
    """Speaker with an ElevenLabs voice; Piper speaks whatever ElevenLabs cannot."""

    def __init__(self, voice_model: str, piper_bin: str = "piper", player: str = "aplay", voice_id: str = VOICE_ID,
                 model: str = TTS_MODEL, timeout: float = 4.0, cache_size: int = 64, client=None, log=None):
        self.client = _client(timeout, client)
        self.voice_id = voice_id
        self.model = model
        self.timeout = timeout
        self.log = log or (lambda msg: None)
        self.fetch = lru_cache(maxsize=cache_size)(self._fetch)  # failures raise, so they are not cached
        super().__init__(voice_model, piper_bin, player)

    def _fetch(self, text: str) -> bytes:
        chunks = self.client.text_to_speech.convert(
            self.voice_id, text=text, model_id=self.model, output_format="wav_22050",
            request_options={"timeout_in_seconds": int(max(1, round(self.timeout))), "max_retries": 0},
        )
        audio = b"".join(chunks)
        if not audio.startswith(b"RIFF"):
            raise ValueError("not a WAV reply")
        return audio

    def synthesize(self, text: str, wav: Path):
        try:
            wav.write_bytes(self.fetch(text))
        except Exception as e:  # no network, no credits, bad key: speak with Piper instead
            self.log(f"ElevenLabs voice failed ({type(e).__name__}: {e}); using Piper")
            super().synthesize(text, wav)


def to_wav_bytes(audio: np.ndarray) -> bytes:
    """16 kHz mono float32 [-1, 1] to a 16-bit WAV file in memory."""
    pcm = (np.clip(audio, -1.0, 1.0) * 32767).astype("<i2")
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        w.writeframes(pcm.tobytes())
    return buf.getvalue()


class ElevenLabsTranscriber:
    """ElevenLabs Scribe; ``fallback`` (a local Transcriber) when the call fails."""

    def __init__(self, fallback=None, model: str = STT_MODEL, timeout: float = 6.0, client=None, log=None):
        self.client = _client(timeout, client)
        self.fallback = fallback
        self.model = model
        self.timeout = timeout
        self.log = log or (lambda msg: None)

    def __call__(self, audio: np.ndarray) -> str:
        if audio.size < SAMPLE_RATE * 0.3:
            return ""
        try:
            result = self.client.speech_to_text.convert(
                model_id=self.model, file=("speech.wav", to_wav_bytes(audio), "audio/wav"), language_code="en",
                tag_audio_events=False, keyterms=ROOM_TERMS,
                request_options={"timeout_in_seconds": int(max(1, round(self.timeout))), "max_retries": 0},
            )
            return (getattr(result, "text", "") or "").strip()
        except Exception as e:
            self.log(f"ElevenLabs transcription failed ({type(e).__name__}: {e}); using faster-whisper")
            return self.fallback(audio) if self.fallback is not None else ""
