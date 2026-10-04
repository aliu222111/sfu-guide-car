"""ElevenLabs: the car's voice and its hearing (spec section 9; ELEVENLABS_API_KEY).

The laptop is on campus WiFi. The car must keep talking when it drops, so every
call has a short timeout; a sentence ElevenLabs cannot voice is spoken by Piper
on the laptop. Mission announcements repeat ("In 5 metres, turn right."), so
synthesized speech is cached by text: it saves credits and the round trip.
Speech-to-text is ElevenLabs Scribe, told the campus's building names and
codes as key terms; without the network the car says it cannot hear just now.

Privacy: the student's recorded voice goes to ElevenLabs; tell them (a sign on
the car).
"""

from __future__ import annotations

import io
import wave
from functools import lru_cache
from pathlib import Path

import numpy as np

from .speech import SAMPLE_RATE, Speaker

TTS_MODEL = "eleven_flash_v2_5"  # the low-latency model
NATURAL_MODEL = "eleven_multilingual_v2"  # slower (about 1 s more) but the most natural voice
RATE = 24000  # pcm_24000: every plan and model offers it
STT_MODEL = "scribe_v1"
VOICE_ID = "21m00Tcm4TlvDq8ikWAM"  # "Rachel", a stock voice; any voice id from the ElevenLabs library works
ROOM_TERMS = ["AQ", "WMC", "MBC", "ASB", "Academic Quadrangle", "West Mall Centre", "Maggie Benston Centre",
              "Applied Science Building", "room",
              "elevator", "door"]


def _client(timeout: float, client=None):
    if client is not None:
        return client
    from elevenlabs.client import ElevenLabs

    return ElevenLabs(timeout=timeout)  # reads ELEVENLABS_API_KEY


_CODES = {"AQ": "A Q", "ASB": "A S B", "WMC": "W M C", "MBC": "M B C", "SCK": "S C K", "TASC": "TASC"}


def for_speech(text: str) -> str:
    """Room codes as people say them: "AQ 3128" -> "A Q 31 28" ("thirty-one twenty-eight"),
    not "ack three thousand one hundred and twenty-eight"; levels ("the 3000 level") stay."""
    import re

    def room(m):
        code, num = m.group(1), m.group(2)
        said = num if len(num) != 4 else f"{num[:2]} {num[2:]}" if num[2] != "0" else f"{num[:2]} oh {num[3]}"
        return f"{_CODES.get(code, code)} {said}"

    return re.sub(r"\b(AQ|ASB|WMC|MBC|SCK|TASC)\s?(\d{3,4})\b", room, text)


class ElevenLabsSpeaker(Speaker):
    """Speaker with an ElevenLabs voice; Piper speaks whatever ElevenLabs cannot."""

    def __init__(self, voice_model: str, device=None, voice_id: str = VOICE_ID,
                 model: str = TTS_MODEL, timeout: float = 4.0, cache_size: int = 64, client=None, log=None):
        self.client = _client(timeout, client)
        self.voice_id = voice_id
        self.model = model
        self.timeout = timeout
        self.log = log or (lambda msg: None)
        self.fetch = lru_cache(maxsize=cache_size)(self._fetch)  # failures raise, so they are not cached
        super().__init__(voice_model, device)

    def _fetch(self, text: str) -> bytes:
        # Raw 16-bit PCM at 24 kHz (every ElevenLabs plan and model offers it), wrapped as WAV here.
        chunks = self.client.text_to_speech.convert(
            self.voice_id, text=for_speech(text), model_id=self.model, output_format=f"pcm_{RATE}",
            request_options={"timeout_in_seconds": int(max(1, round(self.timeout))), "max_retries": 0},
        )
        pcm = b"".join(chunks)
        if len(pcm) < 1000:
            raise ValueError("empty reply")
        buf = io.BytesIO()
        with wave.open(buf, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(pcm)
        return buf.getvalue()

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


class Unheard(Exception):
    """Scribe could not be reached (no network, no credits, bad key)."""


class ElevenLabsTranscriber:
    """ElevenLabs Scribe. ``extra_terms`` adds key terms for one call (a room the car is asking about)."""

    def __init__(self, model: str = STT_MODEL, timeout: float = 6.0, client=None, log=None):
        self.client = _client(timeout, client)
        self.model = model
        self.timeout = timeout
        self.log = log or (lambda msg: None)

    def __call__(self, audio: np.ndarray, extra_terms: tuple[str, ...] = ()) -> str:
        if audio.size < SAMPLE_RATE * 0.3:
            return ""
        try:
            result = self.client.speech_to_text.convert(
                model_id=self.model, file=("speech.wav", to_wav_bytes(audio), "audio/wav"), language_code="en",
                tag_audio_events=False, keyterms=ROOM_TERMS + [t for t in extra_terms if t not in ROOM_TERMS],
                request_options={"timeout_in_seconds": int(max(1, round(self.timeout))), "max_retries": 0},
            )
            return (getattr(result, "text", "") or "").strip()
        except Exception as e:
            self.log(f"ElevenLabs transcription failed ({type(e).__name__}: {e})")
            raise Unheard(str(e)) from e
