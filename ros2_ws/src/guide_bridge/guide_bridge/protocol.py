"""Line protocol to the motor microcontroller (spec section 10).

Downlink, 20 Hz:  ``V <linear m/s> <angular rad/s>\\n``
Uplink, optional: ``E <left ticks> <right ticks>\\n`` from encoders and
``B <0|1>\\n`` from the push-to-talk button on the car (spec section 9).
"""

from __future__ import annotations

from dataclasses import dataclass

# Never send more than this; the microcontroller clamps again.
MAX_LINEAR = 1.2
MAX_ANGULAR = 1.5


@dataclass(frozen=True)
class Encoders:
    left: int
    right: int


@dataclass(frozen=True)
class Button:
    pressed: bool


def _clamp(v: float, lim: float) -> float:
    if v != v:  # NaN
        return 0.0
    return max(-lim, min(lim, v))


def format_velocity(linear: float, angular: float) -> bytes:
    return f"V {_clamp(linear, MAX_LINEAR):.3f} {_clamp(angular, MAX_ANGULAR):.3f}\n".encode("ascii")


def parse_line(line: bytes | str):
    """Parse one uplink line. Returns ``Encoders``, ``Button`` or None for anything else."""
    if isinstance(line, bytes):
        line = line.decode("ascii", errors="replace")
    parts = line.strip().split()
    try:
        if len(parts) == 3 and parts[0] == "E":
            return Encoders(int(parts[1]), int(parts[2]))
        if len(parts) == 2 and parts[0] == "B" and parts[1] in ("0", "1"):
            return Button(parts[1] == "1")
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
