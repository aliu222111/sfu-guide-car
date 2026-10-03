"""Rooms and answers in what speech recognition wrote (spec section 9).

Whisper writes room names the way people say them: "A Q thirty forty six",
"Academic Quadrangle three oh four six", "West Mall twenty-eight thirty",
"MBC 2,290", "8Q 3046". ``rooms_in`` turns a transcript into the destination
keys it can mean ("AQ 3046"), checked against the map, so the voice agent is
told what the student most likely meant, and the car still understands rooms,
yes, no and stop when the language model cannot be reached.
"""

from __future__ import annotations

import re

# How people say, and Whisper writes, each building.
BUILDINGS = {
    "AQ": ("aq", "a q", "a cue", "a queue", "8q", "8 q", "eight q", "academic quadrangle", "academic quad", "the quad"),
    "WMC": ("wmc", "w m c", "west mall centre", "west mall center", "west mall complex", "west mall"),
    "MBC": ("mbc", "m b c", "nbc", "maggie benston centre", "maggie benston center", "maggie benston", "benston"),
}
NAMES = {"AQ": "the Academic Quadrangle", "WMC": "the West Mall Centre", "MBC": "the Maggie Benston Centre"}

UNITS = {"zero": 0, "oh": 0, "o": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
         "eight": 8, "nine": 9}
TEENS = {"ten": 10, "eleven": 11, "twelve": 12, "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16,
         "seventeen": 17, "eighteen": 18, "nineteen": 19}
TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90}

YES = {"yes", "yeah", "yep", "yup", "sure", "ok", "okay", "correct", "right", "open", "done", "ready", "go", "pressed"}
NO = {"no", "nope", "wrong", "not", "cancel"}
STOP = {"stop", "wait", "hold", "pause", "halt"}


def _words(text: str) -> list[str]:
    text = text.lower().replace("-", " ")
    text = re.sub(r"(\d),(\d{3})", r"\1\2", text)  # 3,046
    text = re.sub(r"\b([a-z]{2,3})(\d)", r"\1 \2", text)  # aq3046
    text = re.sub(r"(?<=\b[a-z])\.(?=[a-z]\b)", " ", text)  # a.q.
    return re.findall(r"[a-z]+|\d+(?:\.\d+)?", text)


def _number_runs(words: list[str]) -> list[tuple[int, int, str]]:
    """(start, end, digits) for each spoken number: "thirty forty six" -> "3046"."""
    runs, i = [], 0
    while i < len(words):
        start, parts = i, []
        while i < len(words):
            w = words[i]
            if w[0].isdigit():
                parts.append(w)
            elif w in TENS:
                n = TENS[w]
                if i + 1 < len(words) and words[i + 1] in UNITS and words[i + 1] not in ("oh", "o", "zero"):
                    n += UNITS[words[i + 1]]
                    i += 1
                parts.append(str(n))
            elif w in TEENS:
                parts.append(str(TEENS[w]))
            elif w in UNITS and not (w == "o" and not parts):
                parts.append(str(UNITS[w]))
            elif w == "hundred" and parts:
                parts[-1] += "00" if i + 1 >= len(words) or not _is_number(words[i + 1]) else ""
            elif w == "thousand" and parts:
                parts[-1] += "000" if i + 1 >= len(words) or not _is_number(words[i + 1]) else "0"
            elif w == "point" and parts and i + 1 < len(words) and words[i + 1] in UNITS:
                parts.append("." + str(UNITS[words[i + 1]]))
                i += 1
            else:
                break
            i += 1
        if parts:
            runs.append((start, i, "".join(parts)))
        else:
            i += 1
    return runs


def _is_number(w: str) -> bool:
    return w[0].isdigit() or w in UNITS or w in TEENS or w in TENS


def _buildings(words: list[str]) -> list[tuple[int, int, str]]:
    """(start, end, code) for each building mention."""
    found, joined = [], " ".join(words)
    for code, names in BUILDINGS.items():
        for name in sorted(names, key=len, reverse=True):
            for m in re.finditer(rf"\b{re.escape(name)}\b", joined):
                start = joined[: m.start()].count(" ")
                end = start + name.count(" ") + 1
                if not any(s < end and start < e for s, e, _ in found):
                    found.append((start, end, code))
    return sorted(found)


def rooms_in(text: str, keys) -> list[str]:
    """Destination keys the transcript can mean, most likely first (none: no room in it)."""
    words = _words(text)
    numbers = [r for r in _number_runs(words) if len(r[2].split(".")[0]) >= 3]
    buildings = _buildings(words)
    out = []
    for start, end, digits in numbers:
        # The building said nearest the number, before it ("AQ 3046") or after it ("3046 in AQ").
        near = sorted(buildings, key=lambda b: min(abs(b[1] - start), abs(end - b[0])))
        codes = [near[0][2]] if near else list(BUILDINGS)
        for code in codes:
            key = f"{code} {digits}"
            if key in keys and key not in out:
                out.append(key)
    return out


def answer_in(text: str) -> str | None:
    """"yes", "no" or "stop" when the transcript is one of those (a few words), else None."""
    words = _words(text)
    if not words or len(words) > 6:
        return None
    for kind, vocab in (("stop", STOP), ("no", NO), ("yes", YES)):
        if any(w in vocab for w in words):
            return kind
    return None


def spoken(key: str, dest: dict | None = None) -> str:
    """How the car reads a room back: "AQ 3046, the Multifaith Centre, on the 3000 level of the Academic Quadrangle"."""
    dest = dest or {}
    code = key.partition(" ")[0]
    out = key
    name = (dest.get("name") or "").strip()
    if name and name.lower() not in ("classroom", "office", "room"):
        out += f", the {name}" if not name.lower().startswith("the ") else f", {name}"
    level = (dest.get("floor_name") or "").replace("Level", "level").strip()
    return out + (f", on the {level} of {NAMES.get(code, code)}" if level else f", in {NAMES.get(code, code)}")
