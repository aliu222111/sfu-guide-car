"""Line protocol to the microcontroller that works the RC hand controller (spec section 10).

Downlink, 20 Hz:  ``C <steering -1..1> <throttle -1..1>\\n``, as the
controller's wheel (positive: left) and trigger (positive: forward) would set
them; the laptop works them out (ackermann.py).
Uplink, optional: ``B <0|1>\\n`` from the push-to-talk button on the car (spec section 9) and
``U <cm>\\n`` from the forward ultrasonic sensor (0: nothing in range;
-1: no echo at all, the sensor is missing, dropped so the reading goes stale).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

@dataclass(frozen=True)
class Button:
    pressed: bool


@dataclass(frozen=True)
class Ultrasonic:
    metres: float  # inf when nothing echoed


def _clamp(v: float) -> float:
    if v != v:  # NaN
        return 0.0
    return max(-1.0, min(1.0, v))


def format_drive(steering: float, throttle: float) -> bytes:
    """The downlink line; both clamped to -1..1 (the microcontroller clamps again)."""
    return f"C {_clamp(steering):.3f} {_clamp(throttle):.3f}\n".encode("ascii")


def parse_line(line: bytes | str):
    """Parse one uplink line. Returns ``Button``, ``Ultrasonic`` or None for anything else."""
    if isinstance(line, bytes):
        line = line.decode("ascii", errors="replace")
    parts = line.strip().split()
    try:
        if len(parts) == 2 and parts[0] == "B" and parts[1] in ("0", "1"):
            return Button(parts[1] == "1")
        if len(parts) == 2 and parts[0] == "U":
            cm = int(parts[1])
            if cm < 0:
                return None
            return Ultrasonic(math.inf if cm == 0 else cm / 100.0)
    except ValueError:
        return None
    return None


class LineBuffer:
    """Split a byte stream into lines; drops a line longer than ``limit``."""

    def __init__(self, limit: int = 128):
        self.buf = b""
        self.limit = limit

    def feed(self, data: bytes) -> list[bytes]:
        self.buf += data
        *lines, self.buf = self.buf.split(b"\n")
        if len(self.buf) > self.limit:
            self.buf = b""
        return [l for l in lines if len(l) <= self.limit]
