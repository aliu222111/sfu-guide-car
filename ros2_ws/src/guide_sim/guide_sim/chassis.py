"""The car's hardware below the microcontroller: motor driver, motors, tracks, body.

The simulator reads the firmware's PWM and direction pins through ``Wiring``
(which pin drives which motor, and that the right motor is mounted mirrored,
as on the real car), turns duty cycle into track speed with a motor model
(stiction, a top speed that need not match the firmware's MAX_WHEEL_SPEED, a
time constant) and moves the body with skid-steer kinematics: the tracks slip
sideways, so the car turns slower than the track speed difference says, and a
pivot needs a minimum push before it starts. Nothing here knows about ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Wiring:
    """Pins as wired on the car (match guide_motor.ino) and how the motors are mounted."""

    pwm_left: int = 5
    dir_left: int = 4
    pwm_right: int = 6
    dir_right: int = 7
    button: int = 2
    us_trig: int = 8
    us_echo: int = 9
    # Direction pin level that drives each track forward. The right motor faces
    # the other way on the chassis, so LOW drives it forward (INVERT_R in the firmware).
    left_forward_level: int = 1
    right_forward_level: int = 0


@dataclass
class ChassisParams:
    track_width: float = 0.30  # m between track centres (TRACK_WIDTH_M)
    top_speed: float = 1.1  # m/s track speed at full duty, loaded, on this floor
    stiction_duty: float = 0.22  # below this duty a track does not move: PWM 56, just under the firmware's MIN_PWM 60
    motor_tau: float = 0.12  # s, speed response time constant
    turn_slip: float = 1.45  # actual turn rate = geometric rate / turn_slip
    pivot_breakaway: float = 0.05  # m/s of side-speed difference / 2 needed to start a pivot
    length: float = 0.50  # footprint, matches nav2.yaml
    width: float = 0.38


@dataclass
class BodyState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v_left: float = 0.0  # track speeds, m/s
    v_right: float = 0.0
    v: float = 0.0  # body speed and turn rate after slip
    w: float = 0.0
    collisions: int = 0
    in_collision: bool = False
    history: list = field(default_factory=list)


def track_target(pwm: int, dir_level: int, forward_level: int, p: ChassisParams) -> float:
    """Steady-state track speed for a driver input (m/s, signed)."""
    duty = max(0, min(255, pwm)) / 255.0
    if duty <= p.stiction_duty:
        return 0.0
    speed = (duty - p.stiction_duty) / (1.0 - p.stiction_duty) * p.top_speed
    return speed if dir_level == forward_level else -speed


class Chassis:
    def __init__(self, params: ChassisParams | None = None, wiring: Wiring | None = None):
        self.p = params or ChassisParams()
        self.wiring = wiring or Wiring()
        self.s = BodyState()

    def place(self, x: float, y: float, yaw: float):
        self.s.x, self.s.y, self.s.yaw = x, y, yaw
        self.s.v_left = self.s.v_right = self.s.v = self.s.w = 0.0

    def targets(self, fw) -> tuple[float, float]:
        w, p = self.wiring, self.p
        return (track_target(fw.pwm(w.pwm_left), fw.digital(w.dir_left), w.left_forward_level, p),
                track_target(fw.pwm(w.pwm_right), fw.digital(w.dir_right), w.right_forward_level, p))

    def step(self, dt: float, target_left: float, target_right: float, blocked=None):
        """Advance ``dt`` seconds. ``blocked(x, y, yaw)`` says if a pose collides."""
        p, s = self.p, self.s
        a = 1.0 - math.exp(-dt / p.motor_tau)
        s.v_left += a * (target_left - s.v_left)
        s.v_right += a * (target_right - s.v_right)
        v = (s.v_left + s.v_right) / 2
        half_diff = (s.v_right - s.v_left) / 2
        if abs(v) < 0.03 and abs(half_diff) < p.pivot_breakaway:
            w = 0.0  # the tracks stick: not enough push to drag them sideways
        else:
            w = (s.v_right - s.v_left) / (p.track_width * p.turn_slip)
        yaw = s.yaw + w * dt
        x = s.x + v * math.cos(s.yaw + w * dt / 2) * dt
        y = s.y + v * math.sin(s.yaw + w * dt / 2) * dt
        if blocked is not None and blocked(x, y, yaw):
            # One collision until the car has got 0.2 m clear of where it hit.
            if not s.in_collision:
                s.collisions += 1
                s.history.append((s.x, s.y))
            s.in_collision = True
            s.v = s.w = 0.0
            s.v_left = s.v_right = 0.0  # stalled against the wall
            return
        if s.in_collision and math.hypot(x - s.history[-1][0], y - s.history[-1][1]) > 0.2:
            s.in_collision = False
        s.x, s.y, s.yaw, s.v, s.w = x, y, yaw, v, w

    def footprint(self, x=None, y=None, yaw=None, spacing: float = 0.05) -> list[tuple[float, float]]:
        """Points on the footprint outline in the world frame."""
        s, p = self.s, self.p
        x = s.x if x is None else x
        y = s.y if y is None else y
        yaw = s.yaw if yaw is None else yaw
        hl, hw = p.length / 2, p.width / 2
        pts = []
        nx, ny = max(2, int(p.length / spacing)), max(2, int(p.width / spacing))
        for i in range(nx + 1):
            fx = -hl + p.length * i / nx
            pts += [(fx, hw), (fx, -hw)]
        for j in range(1, ny):
            fy = -hw + p.width * j / ny
            pts += [(hl, fy), (-hl, fy)]
        c, sn = math.cos(yaw), math.sin(yaw)
        return [(x + c * a - sn * b, y + sn * a + c * b) for a, b in pts]
