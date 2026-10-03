"""The RC car's steering and throttle from (speed, turn rate)."""

import math

import pytest

from guide_bridge.ackermann import Ackermann, CarConfig


def test_straight_stop_and_cap():
    car = Ackermann()
    assert car.command(0.0, 0.0) == (0.0, 0.0)
    assert car.command(0.8, 0.0) == (0.0, pytest.approx(0.15 + 0.5 * 0.25))
    assert car.throttle(5.0) == pytest.approx(0.40)  # capped
    assert car.throttle(-0.8) == pytest.approx(-(0.15 + 0.5 * 0.25))


def test_left_turn_follows_the_bicycle_model():
    car = Ackermann()
    steer, _ = car.command(0.5, 0.5)  # 1 m radius
    assert steer == pytest.approx(math.atan(0.48) / 0.52)  # positive: left
    assert car.command(-0.5, -0.5)[0] == pytest.approx(steer)  # backing along the same curve: same wheels


def test_turn_on_the_spot_points_the_wheels_and_full_lock_clamps():
    car = Ackermann()
    assert car.command(0.0, -0.4) == (-1.0, 0.0)  # right lock, not moving
    assert car.command(0.3, 5.0)[0] == 1.0  # tighter than the car can turn: full lock


def test_trim_and_reversed():
    car = Ackermann(CarConfig(steer_trim=0.05, steer_reversed=True, throttle_reversed=True))
    steer, throttle = car.command(0.5, 0.0)
    assert steer == pytest.approx(0.05) and throttle < 0
    assert car.command(0.5, 0.5)[0] < 0
