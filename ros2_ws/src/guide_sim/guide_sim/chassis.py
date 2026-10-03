"""The car's hardware below the microcontroller: hand controller, radio, servo, ESC, body.

The RC car has Ackermann steering and four-wheel drive and is driven through
its stock hand controller, whose wheel and trigger potentiometer wires the
microcontroller drives (PWM through an RC filter). The simulator reads the
firmware's two output pins through ``Wiring`` as wiper voltages, turns them
into the controller's wheel and trigger positions, and the car's receiver into
a front-wheel angle (the servo slews at a finite rate) and throttle (the ESC
has a deadband around neutral and a time constant). The body moves with the
bicycle model, yaw rate = v tan(steer) / wheelbase, with a little understeer.
The true steering trim, full-lock angle, deadband and speed are deliberately
not quite what guide.yaml says, as on a real car before calibration. Nothing
here knows about ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Wiring:
    """Pins as wired (match guide_motor.ino) and the controller's potentiometer voltages."""

    steer: int = 9
    throttle: int = 10
    button: int = 2
    us_trig: int = 11
    us_echo: int = 12
    vref: float = 3.3
    steer_center_v: float = 1.65  # the controller's wheel at rest
    steer_left_v: float = 0.35  # at full left
    throttle_center_v: float = 1.65
    throttle_full_v: float = 2.95  # trigger pulled fully


@dataclass
class ChassisParams:
    wheelbase: float = 0.48  # m, front to rear axle (WHEELBASE_M)
    max_steer: float = 0.50  # rad at full lock: guide.yaml assumes 0.52
    steer_offset: float = 0.03  # wheel position at which it rolls straight: guide.yaml assumes 0
    steer_rate: float = 4.0  # rad/s servo slew
    throttle_start: float = 0.17  # trigger before the wheels turn: guide.yaml assumes 0.15
    throttle_ref: float = 0.40
    speed_at_ref: float = 1.5  # m/s at throttle_ref: guide.yaml assumes 1.6
    top_speed: float = 8.0  # m/s at full trigger (an RC car)
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
        """(wheel speed m/s, steering angle rad) the radio is telling the servo and ESC."""
        w, p = self.wiring, self.p
        volts = lambda pin: max(0, fw.pwm(pin)) / 255.0 * w.vref  # noqa: E731 (the RC filter averages the PWM)
        wheel = max(-1.0, min(1.0, (volts(w.steer) - w.steer_center_v) / (w.steer_left_v - w.steer_center_v)))
        trigger = max(-1.0, min(1.0, (volts(w.throttle) - w.throttle_center_v) / (w.throttle_full_v - w.throttle_center_v)))
        steer = max(-1.0, min(1.0, wheel - p.steer_offset)) * p.max_steer
        speed = 0.0
        if abs(trigger) > p.throttle_start:
            speed = (abs(trigger) - p.throttle_start) / (p.throttle_ref - p.throttle_start) * p.speed_at_ref
            speed = math.copysign(min(speed, p.top_speed), trigger)
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
