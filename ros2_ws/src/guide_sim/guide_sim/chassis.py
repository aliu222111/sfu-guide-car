"""The car's hardware below the microcontroller: steering servo, ESC, motor, body.

The RC car has Ackermann steering and four-wheel drive. The simulator reads the
firmware's two servo pulses through ``Wiring``: the steering servo turns its
pulse into a front-wheel angle (at a finite slew rate), the ESC turns its pulse
into wheel speed (a deadband around neutral, a time constant). The body moves
with the bicycle model, yaw rate = v tan(steer) / wheelbase, with a little
understeer. The true full-lock angle and speed are deliberately not quite what
the firmware's constants say, as on a real car before calibration. Nothing here
knows about ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Wiring:
    """Pins as wired on the car (match guide_motor.ino) and how the servo and ESC respond."""

    steer: int = 9
    esc: int = 10
    button: int = 2
    us_trig: int = 11
    us_echo: int = 12
    steer_center_us: int = 1500  # the pulse at which the wheels point straight
    steer_lock_us: int = 400  # pulse change for full lock
    steer_left_longer: bool = False  # a shorter pulse steers left (STEER_REVERSED false)
    esc_neutral_us: int = 1500
    esc_forward_longer: bool = True  # a longer pulse drives forwards (ESC_REVERSED false)


@dataclass
class ChassisParams:
    wheelbase: float = 0.48  # m, front to rear axle (WHEELBASE_M)
    max_steer: float = 0.50  # rad at full lock: the firmware assumes 0.52
    steer_rate: float = 4.0  # rad/s servo slew
    esc_start_us: int = 70  # beyond neutral before the wheels turn: the firmware assumes 60
    esc_limit_us: int = 180
    speed_at_limit: float = 1.5  # m/s at esc_limit_us: the firmware assumes 1.6
    motor_tau: float = 0.25  # s, speed response time constant (a heavier car than the tank)
    understeer: float = 1.1  # actual yaw rate = geometric / understeer
    length: float = 0.75  # the RC car body, matches nav2.yaml
    width: float = 0.41


@dataclass
class BodyState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0  # body speed, m/s
    w: float = 0.0  # yaw rate, rad/s
    steer: float = 0.0  # front wheel angle, rad, left positive
    collisions: int = 0
    in_collision: bool = False
    history: list = field(default_factory=list)


class Chassis:
    def __init__(self, params: ChassisParams | None = None, wiring: Wiring | None = None):
        self.p = params or ChassisParams()
        self.wiring = wiring or Wiring()
        self.s = BodyState()

    def place(self, x: float, y: float, yaw: float):
        self.s.x, self.s.y, self.s.yaw = x, y, yaw
        self.s.v = self.s.w = 0.0

    def targets(self, fw) -> tuple[float, float]:
        """(wheel speed m/s, steering angle rad) the servo and ESC are being told to reach."""
        w, p = self.wiring, self.p
        steer_us, esc_us = fw.servo_us(w.steer), fw.servo_us(w.esc)
        steer = 0.0
        if steer_us:
            frac = (steer_us - w.steer_center_us) / w.steer_lock_us
            steer = max(-1.0, min(1.0, frac if w.steer_left_longer else -frac)) * p.max_steer
        speed = 0.0
        step = esc_us - w.esc_neutral_us if esc_us else 0  # no pulse: the ESC is disarmed
        if abs(step) > p.esc_start_us:
            frac = min(1.0, (abs(step) - p.esc_start_us) / (p.esc_limit_us - p.esc_start_us))
            speed = math.copysign(frac * p.speed_at_limit, step if w.esc_forward_longer else -step)
        return speed, steer

    def step(self, dt: float, target_speed: float, target_steer: float, blocked=None):
        """Advance ``dt`` seconds. ``blocked(x, y, yaw)`` says if a pose collides."""
        p, s = self.p, self.s
        s.v += (1.0 - math.exp(-dt / p.motor_tau)) * (target_speed - s.v)
        d = target_steer - s.steer
        s.steer += max(-p.steer_rate * dt, min(p.steer_rate * dt, d))
        w = s.v * math.tan(s.steer) / (p.wheelbase * p.understeer)
        yaw = s.yaw + w * dt
        x = s.x + s.v * math.cos(s.yaw + w * dt / 2) * dt
        y = s.y + s.v * math.sin(s.yaw + w * dt / 2) * dt
        if blocked is not None and blocked(x, y, yaw):
            # One collision until the car has got 0.2 m clear of where it hit.
            if not s.in_collision:
                s.collisions += 1
                s.history.append((s.x, s.y))
            s.in_collision = True
            s.v = s.w = 0.0  # stopped against the wall
            return
        if s.in_collision and math.hypot(x - s.history[-1][0], y - s.history[-1][1]) > 0.2:
            s.in_collision = False
        s.x, s.y, s.yaw, s.w = x, y, yaw, w

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
