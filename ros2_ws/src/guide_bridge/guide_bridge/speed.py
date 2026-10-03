"""Closing the RC car's open-loop throttle on measured speed (spec section 10).

An RC ESC sets power, not speed: the speed a given pulse gives changes with the
battery, the floor and the load, and near neutral there is a deadband where the
car does not move at all. The firmware maps speed to pulse with constants
measured once; ``SpeedTrim`` adds a slow correction from the speed the
odometry measures (the local EKF: visual odometry and gyro), so slow speeds
near people are the speeds asked for.

The turn rate is scaled with the speed sent, so the path's curvature, and with
it the steering angle the firmware computes, stays what the planner asked.
A stop is always sent as a stop, and the correction never reverses a command.
"""

from __future__ import annotations

import math

STOP = 0.02  # m/s, the firmware's STOP_SPEED


class SpeedTrim:
    def __init__(self, gain: float = 0.8, limit: float = 0.3, stale: float = 0.5):
        self.gain = gain  # (m/s of trim) per (m/s of error) per second
        self.limit = limit  # m/s
        self.stale = stale  # s: older speed measurements are not used
        self.trim = 0.0
        self.sign = 1.0

    def command(self, linear: float, angular: float, measured: float | None, age: float, dt: float) -> tuple[float, float]:
        """(linear, angular) to send for the wanted (linear, angular)."""
        if abs(linear) < STOP:
            self.trim = 0.0
            return 0.0, angular
        sign = math.copysign(1.0, linear)
        if sign != self.sign:
            self.trim, self.sign = 0.0, sign  # direction changed: the old correction does not apply
        if measured is None or age > self.stale:
            self.trim *= math.exp(-dt / 2.0)  # no measurement: fade the correction out
        else:
            self.trim += self.gain * (linear - measured) * dt
            self.trim = max(-self.limit, min(self.limit, self.trim))
        out = linear + self.trim
        if out * linear <= 0 or abs(out) < STOP:
            out = math.copysign(STOP, linear)  # never stop or reverse the car by correcting
        return out, angular * out / linear
