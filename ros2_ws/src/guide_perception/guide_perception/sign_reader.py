"""Room-number reading from the upper third of the left image (spec sections 7 and 16).

Backends: PaddleOCR on the laptop (default), or Claude Haiku 4.5 in the cloud
(spec section 16, optional) through the owner's Claude subscription
(``ClaudeSubscriptionBackend``, via guide_claude) or an API key
(``ClaudeBackend``). Either returns ``SignRead`` items filtered
to ``[<AQ|WMC|MBC>] <3 or 4 digits>[letter]``.
"""

from __future__ import annotations

import base64
import json
import re
from dataclasses import dataclass

ROOM = re.compile(r"\b(?:(AQ|WMC|MBC)[\s\-]*)?(\d{3,4}[A-Z]?)\b")
# Text that looks like a room number but is not one on a sign.
_NOT_ROOM = re.compile(r"^(19|20)\d\d$")  # years


@dataclass(frozen=True)
class SignRead:
    text: str  # "AQ 3150" or "3150"
    conf: float
    u: float | None = None  # image column of the text's centre, when the backend gives boxes


def extract_rooms(raw: str, conf: float = 1.0) -> list[SignRead]:
    out = []
    for m in ROOM.finditer(raw.upper()):
        building, number = m.group(1), m.group(2)
        if _NOT_ROOM.match(number) and not building:
            continue
        out.append(SignRead(f"{building} {number}" if building else number, conf))
    return out


def upper_third(bgr):
    return bgr[: max(1, bgr.shape[0] // 3)]


def sign_jpeg(bgr) -> bytes | None:
    """The upper third of the frame (where door signs are) as JPEG, for the cloud backends."""
    import cv2

    ok, jpg = cv2.imencode(".jpg", upper_third(bgr), [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpg.tobytes() if ok else None


class PaddleBackend:
    """PaddleOCR 2.x (pinned in requirements-perception.txt)."""

    def __init__(self, min_conf: float = 0.8):
        from paddleocr import PaddleOCR

        self.ocr = PaddleOCR(use_angle_cls=False, lang="en", show_log=False)
        self.min_conf = min_conf

    def read(self, bgr) -> list[SignRead]:
        result = self.ocr.ocr(upper_third(bgr), cls=False) or []
        reads = []
        for page in result:
            for box, (text, conf) in page or []:
                if conf >= self.min_conf:
                    u = sum(p[0] for p in box) / len(box) if box else None
                    reads += [SignRead(r.text, r.conf, u) for r in extract_rooms(text, float(conf))]
        return reads


# Shared by every cloud backend (Claude and Gemini).
CLAUDE_PROMPT = (
    "This is the top of a camera frame from a robot driving down a corridor at Simon Fraser "
    "University. List every room number visible on door signs. Room numbers look like "
    '"3150", "AQ 3150", "WMC 2260" or "MBC 1100A". Answer with JSON only, in the form '
    '{"rooms": ["AQ 3150"]}; use {"rooms": []} if none are readable.'
)


class ClaudeBackend:
    """Sends the frame to Claude Haiku 4.5. Network loss just means no reads (spec section 16)."""

    def __init__(self, model: str = "claude-haiku-4-5", timeout: float = 4.0, client=None):
        if client is None:
            import anthropic

            client = anthropic.Anthropic(timeout=timeout, max_retries=0)
        self.client = client
        self.model = model

    def read(self, bgr) -> list[SignRead]:
        jpg = sign_jpeg(bgr)
        if jpg is None:
            return []
        data = base64.standard_b64encode(jpg).decode("ascii")
        response = self.client.messages.create(
            model=self.model,
            max_tokens=256,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": data}},
                        {"type": "text", "text": CLAUDE_PROMPT},
                    ],
                }
            ],
        )
        if response.stop_reason not in ("end_turn", "stop_sequence"):
            return []
        text = "".join(b.text for b in response.content if b.type == "text")
        return parse_claude_reply(text)


def parse_claude_reply(text: str) -> list[SignRead]:
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        return []
    try:
        rooms = json.loads(text[start : end + 1]).get("rooms", [])
    except (ValueError, AttributeError):
        return []
    reads = []
    for r in rooms if isinstance(rooms, list) else []:
        reads += extract_rooms(str(r), 0.9)
    return reads


class GeminiBackend:
    """Sends the frame to Google Gemini (GEMINI_API_KEY). Same prompt and reply as the Claude backends.

    Use a paid key: on the free tier Google may use the frames, which can show
    students, to improve its products, and the daily request cap runs out in
    well under an hour at one frame a second. Network loss just means no reads.
    """

    def __init__(self, model: str = "gemini-flash-latest", timeout: float = 4.0, client=None):
        if client is None:
            from google import genai
            from google.genai import types

            client = genai.Client(http_options=types.HttpOptions(timeout=int(timeout * 1000)))
        self.client = client
        self.model = model

    def read(self, bgr) -> list[SignRead]:
        from google.genai import types

        jpg = sign_jpeg(bgr)
        if jpg is None:
            return []
        response = self.client.models.generate_content(
            model=self.model,
            contents=[types.Part.from_bytes(data=jpg, mime_type="image/jpeg"), CLAUDE_PROMPT],
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                max_output_tokens=256,
                temperature=0.0,
                thinking_config=types.ThinkingConfig(thinking_budget=0),  # a lookup, not a puzzle: no thinking tokens
            ),
        )
        return parse_claude_reply(response.text or "")


class ClaudeSubscriptionBackend:
    """Claude through the owner's Claude subscription (guide_claude, Claude Agent SDK)."""

    def __init__(self, model: str = "claude-haiku-4-5", reader=None):
        if reader is None:
            from guide_claude.session import use_subscription_env
            from guide_claude.signs import SubscriptionSignReader

            use_subscription_env()
            reader = SubscriptionSignReader(CLAUDE_PROMPT, model=model)
        self.reader = reader

    def read(self, bgr) -> list[SignRead]:
        jpg = sign_jpeg(bgr)
        if jpg is None:
            return []
        return parse_claude_reply(self.reader.read_text(jpg))
