import math

import pytest

from guide_bridge.skid import SpinResult, compensate, mean_rate, new_turn_gain, stall_rate


def test_compensate_scales_only_turn_rate():
    assert compensate(0.4, 0.5, 1.6) == (0.4, pytest.approx(0.8))
    assert compensate(0.4, 0.0, 1.6) == (0.4, 0.0)


def test_slow_pivots_raised_to_min_pivot():
    assert compensate(0.0, 0.1, 1.5, min_pivot=0.25) == (0.0, pytest.approx(0.375))
    assert compensate(0.0, -0.1, 1.0, min_pivot=0.25) == (0.0, pytest.approx(-0.25))
    assert compensate(0.0, 0.02, 1.0, min_pivot=0.25) == (0.0, pytest.approx(0.02))  # deadband
    assert compensate(0.3, 0.1, 1.0, min_pivot=0.25) == (0.3, pytest.approx(0.1))  # driving an arc
    assert compensate(0.0, 0.4, 1.0, min_pivot=0.25) == (0.0, pytest.approx(0.4))


def test_mean_rate_skips_settling():
    samples = [(t * 0.1, 0.0 if t < 15 else 0.3) for t in range(60)]
    assert mean_rate(samples, start=0.0) == pytest.approx(0.3)


def test_gain_from_slipping_spins():
    # Car turns at 60 % of what it is sent, current gain 1.
    spins = [SpinResult(c, 0.6 * c) for c in (0.25, -0.25, 0.45, -0.45)]
    assert new_turn_gain(spins, 1.0) == pytest.approx(1 / 0.6)
    # Run again with that gain: now measured equals commanded, gain stays.
    spins = [SpinResult(c, c) for c in (0.25, -0.25, 0.45, -0.45)]
    assert new_turn_gain(spins, 1 / 0.6) == pytest.approx(1 / 0.6)


def test_stalled_spins_ignored_and_reported():
    spins = [SpinResult(0.15, 0.01), SpinResult(-0.15, -0.02), SpinResult(0.25, 0.2), SpinResult(-0.25, -0.2),
             SpinResult(0.45, 0.36), SpinResult(-0.45, -0.36)]
    assert new_turn_gain(spins, 1.0) == pytest.approx(1.25)
    assert stall_rate(spins) == pytest.approx(0.25)


def test_one_direction_stalling_counts_as_stalled():
    spins = [SpinResult(0.25, 0.25), SpinResult(-0.25, 0.0), SpinResult(0.45, 0.45), SpinResult(-0.45, -0.45)]
    assert stall_rate(spins) == pytest.approx(0.45)


def test_nothing_stalls():
    assert stall_rate([SpinResult(0.2, 0.2), SpinResult(-0.2, -0.2)]) == 0.0


def test_everything_stalls():
    spins = [SpinResult(0.2, 0.0), SpinResult(-0.2, 0.0)]
    assert math.isinf(stall_rate(spins))
    with pytest.raises(ValueError):
        new_turn_gain(spins, 1.0)
