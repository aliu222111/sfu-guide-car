"""The last stop before the motors: every velocity command passes through here.

Applies, in order: source selection (manual creep > recovery rotation > Nav2),
staleness, the mission's enable flag, the person stop and slow-down
(spec section 7), the ultrasonic bumper's stop and slow-down (forward only;
required with one camera, section 18), indoor/outdoor speed caps (section 2),
the 30 deg/s turn cap (section 6) and an acceleration limit. Braking is never
rate limited.

Collision detection: a horizontal jolt in the phone's accelerometer while
moving (the car hit something), or the car told to drive forward while its
measured speed stays near zero (it is pushed up against something, and the
serial bridge's speed trim would keep adding throttle) blocks forward motion
for ``collision_hold`` seconds. Reversing stays allowed so it can back away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class GateParams:
    outdoor_max: float = 1.0
    indoor_max: float = 0.6
    reverse_max: float = 0.2
    turn_max: float = math.radians(30.0)  # over this the car slows down (the curve stays the same)
    accel_max: float = 0.5  # m/s^2, speeding up only
    stale_after: float = 0.5  # s
    person_stop: float = 1.0  # m
    person_slow: float = 3.0  # m
    person_slow_speed: float = 0.25  # m/s cap at person_stop distance
    # rad/s turned on the spot when lost. The Ackermann car cannot turn on the spot,
    # so it stands still while signs and RTAB-Map relocalize from what it sees.
    recovery_rate: float = 0.0
    person_timeout: float = 1.0  # s without a detector message
    require_person_detector: bool = True  # False only for phase 2 tests before perception
    flag_timeout: float = 1.5  # s: enable, recovery and phone liveness must keep arriving
    bumper_stop: float = 0.35  # m: nothing ahead closer than this
    bumper_slow: float = 1.0  # m
    bumper_slow_speed: float = 0.15  # m/s cap at bumper_stop distance
    bumper_timeout: float = 0.5  # s without a reading
    require_bumper: bool = False  # True with one camera: no reading, creep
    jolt_stop: float = 5.0  # m/s^2 horizontal, above the slow trend: a bump (braking gives ~2)
    jolt_min_speed: float = 0.1  # m/s commanded: only while moving
    stall_cmd: float = 0.15  # m/s commanded forward ...
    stall_speed: float = 0.04  # ... while measured speed stays under this ...
    stall_time: float = 1.2  # s ... this long: blocked
    collision_hold: float = 3.0  # s with forward motion blocked
    odom_timeout: float = 0.5  # s: an older speed reading cannot show a stall


@dataclass
class _Cmd:
    linear: float = 0.0
    angular: float = 0.0
    stamp: float = -math.inf


@dataclass
class _Flag:
    """A boolean input that only counts while it keeps being re-sent."""

    value: bool = False
    stamp: float = -math.inf

    def get(self, now: float, timeout: float, default: bool = False) -> bool:
        return self.value if now - self.stamp <= timeout else default


@dataclass
class GateState:
    nav: _Cmd = field(default_factory=_Cmd)
    manual: _Cmd = field(default_factory=_Cmd)
    enabled: _Flag = field(default_factory=_Flag)
    recovery: _Flag = field(default_factory=_Flag)
    outdoor: _Flag = field(default_factory=_Flag)  # stale (localization manager died): indoor caps
    phone_alive: _Flag = field(default_factory=_Flag)
    person_distance: float = math.inf
    person_stamp: float = -math.inf
    bumper_distance: float = math.inf
    bumper_stamp: float = -math.inf
    last_linear: float = 0.0
    last_time: float | None = None
    accel_trend: tuple = (0.0, 0.0)  # slow average of the horizontal acceleration (tilt, bias)
    accel_stamp: float | None = None
    speed: float = 0.0
    speed_stamp: float = -math.inf
    stall_since: float | None = None
    collision_until: float = -math.inf
    collision_kind: str = ""
    collisions: int = 0


def _finite(*vals) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


class VelocityGate:
    """Every input is timestamped. The mission re-sends enable and recovery at
    1 Hz, the phone bridge sends liveness and the localization manager the
    outdoor flag at 5 Hz; when any of them stops arriving (a node crashed) the
    gate falls back to the safe value: disabled, no recovery rotation, phone
    treated as lost, indoor speed."""

    def __init__(self, params: GateParams | None = None):
        self.p = params or GateParams()
        self.s = GateState()

    # Inputs -------------------------------------------------------------------
    def nav_cmd(self, linear, angular, now):
        self.s.nav = _Cmd(linear, angular, now) if _finite(linear, angular) else _Cmd(0.0, 0.0, -math.inf)

    def manual_cmd(self, linear, angular, now):
        self.s.manual = _Cmd(linear, angular, now) if _finite(linear, angular) else _Cmd(0.0, 0.0, -math.inf)

    def set_enabled(self, on: bool, now: float):
        self.s.enabled = _Flag(bool(on), now)

    def set_recovery(self, on: bool, now: float):
        self.s.recovery = _Flag(bool(on), now)

    def set_outdoor(self, on: bool, now: float):
        self.s.outdoor = _Flag(bool(on), now)

    def set_phone_alive(self, on: bool, now: float):
        self.s.phone_alive = _Flag(bool(on), now)

    def person(self, distance: float, now: float):
        self.s.person_distance = distance if distance == distance else 0.0  # NaN: assume the worst
        self.s.person_stamp = now

    def bumper(self, distance: float, now: float):
        """Ultrasonic range ahead, metres; inf when nothing echoes."""
        self.s.bumper_distance = distance if distance == distance else 0.0  # NaN: assume the worst
        self.s.bumper_stamp = now

    def accel(self, ax: float, ay: float, now: float):
        """Phone acceleration in base_link axes (m/s^2): x forward, y left."""
        if not _finite(ax, ay):
            return
        s, p = self.s, self.p
        tx, ty = s.accel_trend
        if s.accel_stamp is None:
            s.accel_trend, s.accel_stamp = (ax, ay), now
            return
        jolt = math.hypot(ax - tx, ay - ty)
        a = min(1.0, max(0.0, now - s.accel_stamp) / 1.0)  # 1 s trend
        s.accel_trend, s.accel_stamp = (tx + a * (ax - tx), ty + a * (ay - ty)), now
        if jolt > p.jolt_stop and abs(s.last_linear) >= p.jolt_min_speed:
            self._collided(now, f"jolt {jolt:.1f} m/s^2")

    def odom_speed(self, v: float, now: float):
        """Measured forward speed, m/s (the EKF's /odometry/local)."""
        if _finite(v):
            self.s.speed, self.s.speed_stamp = v, now

    def _collided(self, now: float, kind: str):
        s = self.s
        if now >= s.collision_until:
            s.collisions += 1
        s.collision_until = now + self.p.collision_hold
        s.collision_kind = kind
        s.stall_since = None

    def _check_stall(self, now: float):
        s, p = self.s, self.p
        pushing = s.last_linear >= p.stall_cmd and now - s.speed_stamp <= p.odom_timeout and abs(s.speed) < p.stall_speed
        if not pushing:
            s.stall_since = None
        elif s.stall_since is None:
            s.stall_since = now
        elif now - s.stall_since >= p.stall_time:
            self._collided(now, "stalled")

    def collided(self, now: float) -> bool:
        return now < self.s.collision_until

    # Output -------------------------------------------------------------------
    def speed_cap(self, now: float) -> float:
        p = self.p
        phone = self.s.phone_alive.get(now, p.flag_timeout)
        outdoor = self.s.outdoor.get(now, p.flag_timeout)
        cap = p.outdoor_max if (outdoor and phone) else p.indoor_max
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

    def forward_cap(self, now: float, cap: float) -> float:
        """Forward speed limit from the bumper (reversing and turning are not limited)."""
        p = self.p
        d = self.s.bumper_distance
        if now - self.s.bumper_stamp > p.bumper_timeout:
            if p.require_bumper:
                return min(cap, p.bumper_slow_speed)
            return cap
        if d < p.bumper_stop:
            return 0.0
        if d < p.bumper_slow:
            frac = (d - p.bumper_stop) / (p.bumper_slow - p.bumper_stop)
            return min(cap, p.bumper_slow_speed + frac * (cap - p.bumper_slow_speed))
        return cap

    def step(self, now: float) -> tuple[float, float, str]:
        """Command to send now: (linear m/s, angular rad/s, reason)."""
        p, s = self.p, self.s
        dt = 0.0 if s.last_time is None else max(0.0, now - s.last_time)
        s.last_time = now

        if not s.enabled.get(now, p.flag_timeout):
            return self._out(0.0, 0.0, "disabled")
        if now - s.manual.stamp <= p.stale_after:
            lin, ang, why = s.manual.linear, s.manual.angular, "manual"
        elif s.recovery.get(now, p.flag_timeout):
            lin, ang, why = 0.0, p.recovery_rate, "recovery"
        elif now - s.nav.stamp <= p.stale_after:
            lin, ang, why = s.nav.linear, s.nav.angular, "nav"
        else:
            return self._out(0.0, 0.0, "stale")

        self._check_stall(now)
        if self.collided(now) and lin > 0:
            return self._out(0.0, 0.0, "collision")  # forward blocked; reversing away is allowed

        wanted = lin
        cap = self.speed_cap(now)
        if cap == 0.0:
            return self._out(0.0, 0.0, "person")
        fwd = self.forward_cap(now, cap)
        if lin > fwd:
            lin = fwd
            if fwd == 0.0:
                why = "bumper"
        lin = max(-p.reverse_max, min(cap, lin))
        last = s.last_linear
        if lin * last < 0:
            # Reversing: stop for one tick, then ramp up from zero (no turn either: the
            # Ackermann bridge would slam the wheels to full lock for that tick).
            return self._out(0.0, 0.0, why)
        if abs(lin) > abs(last):
            # Rate limit speeding up only; braking is never delayed.
            max_step = p.accel_max * dt if dt > 0 else 0.05
            lin = math.copysign(min(abs(lin), abs(last) + max_step), lin)
        # Ackermann steering: the turn rate follows the speed, so a slower car
        # drives the same curve (steering angle = atan(wheelbase * w / v)).
        if wanted != 0.0:
            ang *= lin / wanted
        if abs(ang) > p.turn_max:  # too fast for this curve: slow down, keep the curve
            k = p.turn_max / abs(ang)
            lin, ang = lin * k, ang * k
        return self._out(lin, ang, why)

    def _out(self, lin, ang, why):
        self.s.last_linear = lin
        return lin, ang, why
