"""Speed trim for the open-loop RC throttle."""

import pytest

from guide_bridge.speed import SpeedTrim


def run(trim, target, true_speed_of, seconds, dt=0.05, measured=True):
    v = 0.0
    out = (0.0, 0.0)
    for _ in range(int(seconds / dt)):
        out = trim.command(target, 0.5 * target, v if measured else None, 0.1, dt)
        v = true_speed_of(out[0])
    return v, out


def test_slow_car_is_brought_up_to_speed_and_keeps_its_curvature():
    car = lambda sent: max(0.0, sent - 0.08)  # ESC deadband the firmware does not know about
    v, (lin, ang) = run(SpeedTrim(), 0.15, car, 6.0)
    assert v == pytest.approx(0.15, abs=0.01)
    assert ang / lin == pytest.approx(0.5)  # curvature unchanged: same steering angle


def test_fast_car_is_slowed_down():
    v, _ = run(SpeedTrim(), 0.3, lambda sent: 1.3 * sent, 6.0)
    assert v == pytest.approx(0.3, abs=0.01)


def test_stop_is_a_stop_and_trim_is_bounded():
    t = SpeedTrim(limit=0.3)
    run(t, 0.2, lambda sent: 0.0, 10.0)  # wheels blocked: the trim saturates at its small-speed cap
    assert t.trim == pytest.approx(0.12)
    assert t.command(0.0, 0.3, 0.0, 0.1, 0.05) == (0.0, 0.3)
    assert t.trim == 0.0


def test_no_measurement_fades_out_and_never_reverses():
    t = SpeedTrim()
    t.trim = -0.3
    lin, _ = t.command(0.1, 0.0, None, 9.9, 0.05)
    assert lin > 0  # a correction never reverses the car
    for _ in range(200):
        t.command(0.1, 0.0, None, 9.9, 0.05)
    assert abs(t.trim) < 0.01


def test_direction_change_resets():
    t = SpeedTrim()
    run(t, 0.2, lambda sent: max(0.0, sent - 0.1), 4.0)
    assert t.trim > 0.05
    t.command(-0.2, 0.0, 0.0, 0.1, 0.05)
    assert abs(t.trim) < 0.02
