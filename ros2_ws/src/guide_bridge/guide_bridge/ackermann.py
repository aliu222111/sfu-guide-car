"""The RC car's steering and throttle, worked out on the laptop (spec section 10).

The car is a stock RC car with Ackermann steering and four-wheel drive. A
microcontroller stands in for the hand controller's steering wheel and trigger
(it drives their potentiometer wires), so the laptop sends it two numbers,
steering and throttle, each from -1 to 1, as the controller's own wheel and
trigger would set them. Everything about the car lives here, in guide.yaml,
not in the microcontroller: the bicycle model, steer = atan(wheelbase * turn
rate / speed), the throttle deadband around neutral and a throttle cap (RC
cars are fast). A car cannot turn without moving: asked to turn while stopped,
it points the wheels and waits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class CarConfig:
    """Measured on the floor (README runbook, "The RC car")."""

    wheelbase: float = 0.48  # m, front to rear axle
    max_steer: float = 0.52  # rad at full lock: from a slow full-lock circle of radius R, atan(wheelbase / R)
    steer_trim: float = 0.0  # steering value at which the car rolls straight
    steer_reversed: bool = False  # true if a positive steering value turns right
    throttle_start: float = 0.15  # trigger at which the car just starts to roll
    throttle_limit: float = 0.40  # never more trigger than this, either way
    speed_at_limit: float = 1.6  # m/s at throttle_limit on the floor
    throttle_reversed: bool = False  # true if a positive throttle drives backwards
    stop_speed: float = 0.02  # m/s: slower requests are a stop (the speed trim uses it too)


class Ackermann:
    def __init__(self, cfg: CarConfig | None = None):
        self.c = cfg or CarConfig()
        self.steer = 0.0  # rad, held while stopped so the car starts into a turn

    def steering(self, v: float, w: float) -> float:
        c = self.c
        if abs(v) >= c.stop_speed:
            self.steer = math.atan(c.wheelbase * w / v)  # reversing: the same formula steers the right way
        elif abs(w) > 0.05:
            self.steer = math.copysign(c.max_steer, w)  # asked to turn on the spot: point the wheels
        self.steer = max(-c.max_steer, min(c.max_steer, self.steer))
        s = self.steer / c.max_steer  # positive: left
        return max(-1.0, min(1.0, c.steer_trim + (-s if c.steer_reversed else s)))

    def throttle(self, v: float) -> float:
        c = self.c
        if abs(v) < c.stop_speed:
            return 0.0
        f = min(1.0, abs(v) / c.speed_at_limit)
        t = c.throttle_start + f * (c.throttle_limit - c.throttle_start)
        forward = (v > 0) != c.throttle_reversed
        return t if forward else -t

    def command(self, v: float, w: float) -> tuple[float, float]:
        """(steering, throttle), each -1..1, for speed v (m/s) and turn rate w (rad/s)."""
        return self.steering(v, w), self.throttle(v)
