"""Rooms and answers in what speech recognition wrote (spec section 9).

Scribe writes room names the way people say them: "A Q thirty forty six",
"Academic Quadrangle three oh four six", "West Mall twenty-eight thirty",
"MBC 2,290", "8Q 3046". ``rooms_in`` turns a transcript into the destination
keys it can mean ("AQ 3046"), checked against the map, so the voice agent is
told what the student most likely meant, and the car still understands rooms,
yes, no and stop when the language model cannot be reached.

``understand`` also says when a room that was said is not on the map (and
which real rooms are closest: a digit misheard or swapped, the same number in
another building), and finds places said by name or kind ("the Multifaith
Centre", "a washroom": the nearest one). ``suggestion`` words the reply.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field

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


def _said(text: str) -> list[tuple[list[str], str]]:
    """(building codes it can be in, digits) for each room number said."""
    words = _words(text)
    buildings = _buildings(words)
    out = []
    for start, end, digits in _number_runs(words):
        if len(digits.split(".")[0]) < 3:
            continue
        # The building said nearest the number, before it ("AQ 3046") or after it ("3046 in AQ").
        near = sorted(buildings, key=lambda b: min(abs(b[1] - start), abs(end - b[0])))
        out.append(([near[0][2]] if near else list(BUILDINGS), digits))
    return out


def rooms_in(text: str, keys) -> list[str]:
    """Destination keys the transcript can mean, most likely first (none: no room in it)."""
    out = []
    for codes, digits in _said(text):
        for code in codes:
            key = f"{code} {digits}"
            if key in keys and key not in out:
                out.append(key)
    return out


def _one_edit(a: str, b: str) -> float:
    """0 equal, 1 two digits swapped ("sixty four" for "forty six"), 1.5 one digit changed,
    2 one digit added or dropped, 9 anything else."""
    if a == b:
        return 0
    if len(a) == len(b):
        diff = [i for i in range(len(a)) if a[i] != b[i]]
        if len(diff) == 1:
            return 1.5
        if len(diff) == 2 and diff[1] == diff[0] + 1 and a[diff[0]] == b[diff[1]] and a[diff[1]] == b[diff[0]]:
            return 1
        return 9
    short, long_ = sorted((a, b), key=len)
    if len(long_) - len(short) == 1 and any(long_[:i] + long_[i + 1:] == short for i in range(len(long_))):
        return 2
    return 9


def _number(key: str) -> str:
    return key.partition(" ")[2]


def suggest_rooms(codes, digits: str, keys, limit: int = 2) -> list[str]:
    """Real rooms closest to one that is not on the map, most likely first."""
    scored = []
    for key in keys:
        code, number = key.partition(" ")[0], _number(key)
        if "." in number:  # "AQ 2003.1": a room inside a room, not a door anyone reads out
            continue
        if number == digits:
            score = 0  # the same number in another building
        elif code not in codes:
            continue
        else:
            score = _one_edit(digits, number)
            if score == 9 and number[:1] == digits[:1] and len(number) == len(digits):
                lead = re.match(r"\d+", number)
                gap = abs(int(lead.group()) - int(digits)) if lead and digits.isdigit() else 999
                score = 3 if gap <= 10 else 9  # a door a few numbers along the same corridor
            if score == 9:
                continue
        lead = re.match(r"\d+", number)
        gap = abs(int(lead.group()) - int(digits)) if lead and digits.isdigit() else 0
        scored.append((score, gap, key))
    return [k for *_, k in sorted(scored)[:limit]]


# What people call a kind of place, and the destination types it means.
KINDS = {
    "washroom": (("washroom", "bathroom", "restroom", "toilet", "toilets", "loo", "washrooms", "bathrooms"), ("Washroom",)),
    "lecture theatre": (("lecture theatre", "lecture theater", "lecture hall", "theatre", "theater"), ("Lecture Theatre",)),
    "classroom": (("classroom",), ("Classroom",)),
    "study space": (("study area", "study space", "study room", "somewhere to study", "place to study"), ("Public Study Area",)),
    "place to eat": (("food", "eat", "coffee", "cafe", "lunch", "something to eat", "drink", "juice", "dining"),
                     ("Dining", "Food Service")),
    "computer lab": (("computer lab", "computer laboratory", "computers"), ("Computer Laboratory",)),
}
STOP_WORDS = {"the", "a", "an", "of", "to", "for", "and", "in", "on", "at", "office", "centre", "center", "room",
              "take", "me", "please", "go", "i", "want", "need", "find", "where", "is", "can", "you", "my", "s"}


def _place_words(name: str) -> list[str]:
    return [w for w in _words(name.replace("&", " and ")) if w not in STOP_WORDS and len(w) > 1]


def places_in(text: str, destinations: dict, here: dict | None = None, limit: int = 2) -> tuple[list[str], str]:
    """Destinations named in the transcript ("Multifaith Centre"), or the nearest of a kind said
    ("a washroom"), and that kind ("" for names). ``here``: {"floor": "AQ-03", "pose": [x, y, ...]}."""
    words = _words(text)
    said = set(words)
    joined = "".join(words)
    text_l = " ".join(words)
    kind = next((k for k, (phrases, _) in KINDS.items()
                 if any(re.search(rf"\b{re.escape(p)}\b", text_l) for p in phrases)), "")
    named = []
    for key, d in destinations.items():
        name = (d.get("name") or "").strip()
        nw = _place_words(name)
        if not nw or name.upper() == name and len(nw) < 2:  # "TA OFFICE" and the like: not a place to ask for
            continue
        hit = sum(w in said for w in nw) / len(nw)
        if "".join(nw) in joined:
            hit = 1.0  # "multi faith" for "Multifaith"
        if hit >= (1.0 if kind else 0.5):  # "women's washroom" is a washroom, not Women's Studies
            named.append((-hit, key))
    if named:
        best = min(h for h, _ in named)
        return [k for h, k in sorted(named) if h == best][:limit], ""
    text_l = " ".join(words)
    if kind:
        phrases, types = KINDS[kind]
        cands = [k for k, d in destinations.items()
                 if any((d.get("type") or "").startswith(t) for t in types) and "." not in _number(k)
                 or set(_place_words(d.get("name") or "")) & set(phrases)]  # "Renaissance Coffee" is a place to eat
        if kind == "washroom":
            for gender in ("women", "men"):
                if gender in said:
                    cands = [k for k in cands if re.search(rf"\b{gender}'s", destinations[k]["type"])
                             or "single" in destinations[k]["type"]]
                    break
        codes = {c for *_, c in _buildings(words)}
        if codes:
            cands = [k for k in cands if destinations[k].get("building") in codes]
        floor = (here or {}).get("floor")
        pose = (here or {}).get("pose")

        def far(k, floor=floor, pose=pose):
            d = destinations[k]
            if floor and d.get("floor") == floor and pose and "room_x" in d:
                return math.hypot(d["room_x"] - pose[0], d["room_y"] - pose[1])
            if floor and d.get("building") == floor.split("-")[0]:
                return 1000 + 100 * abs(_level(d.get("floor")) - _level(floor))
            return 10000

        if floor or codes:
            # A place named for what was said ("coffee": Renaissance Coffee) before the nearest of the kind.
            return sorted(cands, key=lambda k: (not set(_place_words(destinations[k].get("name") or "")) & said,
                                                far(k), k))[:1], kind
        return [], kind  # no idea where the car is: ask which building
    return [], ""


def _level(floor: str | None) -> int:
    m = re.search(r"(\d+)", (floor or "").partition("-")[2])
    return int(m.group(1)) if m else 0


@dataclass
class Heard:
    rooms: list = field(default_factory=list)  # rooms said that are on the map
    missing: str = ""  # a room said that is not ("AQ 3064")
    suggestions: list = field(default_factory=list)  # real rooms or places it may have meant
    kind: str = ""  # a kind of place asked for ("washroom")


def understand(text: str, destinations: dict, here: dict | None = None) -> Heard:
    rooms = rooms_in(text, destinations)
    if rooms:
        return Heard(rooms=rooms)
    said = _said(text)
    if said:
        codes, digits = said[0]
        missing = f"{codes[0]} {digits}" if len(codes) == 1 else f"room {digits}"
        return Heard(missing=missing, suggestions=suggest_rooms(codes, digits, destinations))
    places, kind = places_in(text, destinations, here)
    return Heard(suggestions=places, kind=kind)


def suggestion(h: Heard, destinations: dict) -> tuple[str, str | None]:
    """What the car says about a room not on the map or a place asked for, and the destination it
    proposes (to start on a yes), or ("", None) when there is nothing to say."""
    def where(k):
        d = dict(destinations.get(k) or {})
        kind = d.get("type") or ""
        if not d.get("name") and kind.startswith("Washroom"):  # "Washroom, men's, accessible"
            parts = kind.split(", ")
            d["name"] = " ".join(parts[1:] + [parts[0].lower()])
        return spoken(k, d)

    if h.missing:
        lead = f"I can't find {h.missing} on my map."
        if len(h.suggestions) == 1:
            return f"{lead} Did you mean {where(h.suggestions[0])}?", h.suggestions[0]
        if h.suggestions:
            return f"{lead} Did you mean {' or '.join(h.suggestions)}?", None
        return f"{lead} Could you check the room number on your timetable?", None
    if h.kind and not h.suggestions:
        return f"Which building should I look for a {h.kind} in: AQ, the West Mall Centre or the Maggie Benston Centre?", None
    if len(h.suggestions) == 1:
        k = h.suggestions[0]
        if h.kind:
            return f"The nearest {h.kind} I know is {where(k)}. Shall we go there?", k
        return f"Do you mean {where(k)}?", k
    if h.suggestions:
        names = [f"{destinations[k].get('name') or k}, {k}" for k in h.suggestions]
        return f"Did you mean {' or '.join(names)}?", None
    return "", None


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
