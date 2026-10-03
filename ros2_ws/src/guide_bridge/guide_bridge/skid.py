"""Tank (skid-steer) chassis: turn-slip compensation and its calibration.

A tracked or four-wheel skid-steer car turns by driving its sides at
different speeds and dragging the tracks sideways. The drag means it turns
slower than the geometry says: commanding 0.5 rad/s with the textbook mix
(side speeds v -+ w * track / 2) gives perhaps 0.3 rad/s on carpet. The
serial bridge multiplies the commanded turn rate by ``turn_gain`` to make up
for it, and ``calibrate_turn`` measures the gain with the phone gyro. Below
some turn rate a pivot in place does not start at all (the tracks stick).
Nav2 asks for such slow pivots near waypoints, and the car would sit still
until the progress checker gives up, so the bridge raises any deliberate
pivot below ``min_pivot`` to ``min_pivot`` (also measured by
``calibrate_turn``). The firmware's MIN_PWM_PIVOT is the motor-level version of the
same fix.

Nothing here trusts wheel or track motion for heading: skid-steer slip makes
encoder yaw useless, so heading comes from the phone gyro, visual odometry
and the vanishing point (spec section 6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PIVOT_SPEED = 0.02  # m/s: slower than this the car is pivoting (the gate's first 0.025 ramp step is not)
PIVOT_DEADBAND = 0.05  # rad/s: smaller turn requests are noise, not worth a jerk to min_pivot
SPIN_SETTLE = 1.5  # s ignored at the start of each spin while the tracks break loose
STALLED = 0.5  # measured below this fraction of the command: the pivot did not really start


def compensate(linear: float, angular: float, turn_gain: float, min_pivot: float = 0.0) -> tuple[float, float]:
    """The command to send for a wanted (linear, angular).

    Pivots slower than ``min_pivot`` (rad/s, in the planner's units) are
    raised to it; then the turn rate is scaled by the slip gain.
    """
    if abs(linear) < PIVOT_SPEED and PIVOT_DEADBAND <= abs(angular) < min_pivot:
        angular = math.copysign(min_pivot, angular)
    return linear, angular * turn_gain


@dataclass
class SpinResult:
    commanded: float  # rad/s asked of the car (after the bridge's current gain)
    measured: float  # rad/s from the gyro, mean after settling


def mean_rate(samples: list[tuple[float, float]], start: float, settle: float = SPIN_SETTLE) -> float:
    """Mean gyro z rate of ``(t, wz)`` samples taken at or after ``start + settle``."""
    rates = [w for t, w in samples if t >= start + settle]
    return sum(rates) / len(rates) if rates else 0.0


def stall_rate(results: list[SpinResult]) -> float:
    """Smallest commanded rate whose spins (both directions) all really turned; 0 if none stalled."""
    by_rate: dict[float, bool] = {}
    for r in results:
        turned = r.commanded != 0 and r.measured / r.commanded >= STALLED
        key = round(abs(r.commanded), 3)
        by_rate[key] = by_rate.get(key, True) and turned
    stalled = [rate for rate, ok in by_rate.items() if not ok]
    if not stalled:
        return 0.0
    moving = sorted(rate for rate, ok in by_rate.items() if ok and rate > max(stalled))
    return moving[0] if moving else float("inf")


def new_turn_gain(results: list[SpinResult], current_gain: float) -> float:
    """Gain that makes the measured rate match the request, from the spins that did not stall.

    ``commanded`` is what the planner asked for; the bridge sent ``commanded *
    current_gain`` to the motors. The car turned ``measured``. A gain of
    ``current_gain * commanded / measured`` would have turned it at
    ``commanded``. The median over spins resists one bad spin.
    """
    ratios = sorted(
        r.commanded / r.measured
        for r in results
        if r.commanded != 0 and r.measured / r.commanded >= STALLED
    )
    if not ratios:
        raise ValueError("every spin stalled: raise MIN_PWM_PIVOT in the firmware or test on a smoother floor")
    mid = len(ratios) // 2
    median = ratios[mid] if len(ratios) % 2 else (ratios[mid - 1] + ratios[mid]) / 2
    return current_gain * median
