"""The last stop before the motors: every velocity command passes through here.

Applies, in order: source selection (manual creep > recovery rotation > Nav2),
staleness, the mission's enable flag, the person stop and slow-down
(spec section 7), indoor/outdoor speed caps (section 2), the 30 deg/s turn cap
(section 6) and an acceleration limit. Braking is never rate limited.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class GateParams:
    outdoor_max: float = 1.0
    indoor_max: float = 0.6
    reverse_max: float = 0.2
    turn_max: float = math.radians(30.0)
    accel_max: float = 0.5  # m/s^2, speeding up only
    stale_after: float = 0.5  # s
    person_stop: float = 1.0  # m
    person_slow: float = 3.0  # m
    person_slow_speed: float = 0.25  # m/s cap at person_stop distance
    recovery_rate: float = math.radians(15.0)  # rad/s, slow rotation when lost
    person_timeout: float = 1.0  # s without a detector message
    require_person_detector: bool = True  # False only for phase 2 tests before perception


@dataclass
class _Cmd:
    linear: float = 0.0
    angular: float = 0.0
    stamp: float = -math.inf


@dataclass
class GateState:
    nav: _Cmd = field(default_factory=_Cmd)
    manual: _Cmd = field(default_factory=_Cmd)
    enabled: bool = False
    recovery: bool = False
    outdoor: bool = False
    phone_alive: bool = True
    person_distance: float = math.inf
    person_stamp: float = -math.inf
    last_linear: float = 0.0
    last_time: float | None = None


class VelocityGate:
    def __init__(self, params: GateParams | None = None):
        self.p = params or GateParams()
        self.s = GateState()

    # Inputs -------------------------------------------------------------------
    def nav_cmd(self, linear, angular, now):
        self.s.nav = _Cmd(linear, angular, now)

    def manual_cmd(self, linear, angular, now):
        self.s.manual = _Cmd(linear, angular, now)

    def set_enabled(self, on: bool):
        self.s.enabled = bool(on)

    def set_recovery(self, on: bool):
        self.s.recovery = bool(on)

    def set_outdoor(self, on: bool):
        self.s.outdoor = bool(on)

    def set_phone_alive(self, on: bool):
        self.s.phone_alive = bool(on)

    def person(self, distance: float, now: float):
        self.s.person_distance = distance if distance == distance else math.inf
        self.s.person_stamp = now

    # Output -------------------------------------------------------------------
    def speed_cap(self, now: float) -> float:
        p = self.p
        cap = p.outdoor_max if (self.s.outdoor and self.s.phone_alive) else p.indoor_max
        d = self.s.person_distance
        if now - self.s.person_stamp > p.person_timeout:
            # No word from the person detector: it may be blind, so creep.
            if p.require_person_detector:
                return min(cap, p.person_slow_speed)
            d = math.inf
        if d < p.person_stop:
            return 0.0
        if d < p.person_slow:
            frac = (d - p.person_stop) / (p.person_slow - p.person_stop)
            cap = min(cap, p.person_slow_speed + frac * (cap - p.person_slow_speed))
        return cap

    def step(self, now: float) -> tuple[float, float, str]:
        """Command to send now: (linear m/s, angular rad/s, reason)."""
        p, s = self.p, self.s
        dt = 0.0 if s.last_time is None else max(0.0, now - s.last_time)
        s.last_time = now

        if not s.enabled:
            return self._out(0.0, 0.0, "disabled")
        if now - s.manual.stamp <= p.stale_after:
            lin, ang, why = s.manual.linear, s.manual.angular, "manual"
        elif s.recovery:
            lin, ang, why = 0.0, p.recovery_rate, "recovery"
        elif now - s.nav.stamp <= p.stale_after:
            lin, ang, why = s.nav.linear, s.nav.angular, "nav"
        else:
            return self._out(0.0, 0.0, "stale")

        cap = self.speed_cap(now)
        if cap == 0.0:
            return self._out(0.0, 0.0, "person")
        lin = max(-p.reverse_max, min(cap, lin))
        ang = max(-p.turn_max, min(p.turn_max, ang))
        # Rate limit speeding up only, in either direction of travel.
        if abs(lin) > abs(s.last_linear) and math.copysign(1, lin) == math.copysign(1, s.last_linear or lin):
            max_step = p.accel_max * dt if dt > 0 else 0.05
            lin = math.copysign(min(abs(lin), abs(s.last_linear) + max_step), lin)
        return self._out(lin, ang, why)

    def _out(self, lin, ang, why):
        self.s.last_linear = lin
        return lin, ang, why
