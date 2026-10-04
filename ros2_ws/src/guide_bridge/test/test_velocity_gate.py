import math

import pytest

from guide_bridge.velocity_gate import GateParams, VelocityGate


def gate(**kw):
    return VelocityGate(GateParams(require_person_detector=False, **kw))


def tick(g, t, lin, ang=0.0, outdoor=False, phone=True):
    """One 50 ms control tick with the mission and phone bridge alive."""
    g.set_enabled(True, t)
    g.set_outdoor(outdoor, t)
    if phone:
        g.set_phone_alive(True, t)
    g.nav_cmd(lin, ang, t)
    return g.step(t)


def ramp(g, t0, lin, ang=0.0, secs=5.0, **kw):
    t, out = t0, None
    while t < t0 + secs:
        out = tick(g, t, lin, ang, **kw)
        t += 0.05
    return out


def test_disabled_and_stale_stop():
    g = VelocityGate()
    g.nav_cmd(0.5, 0, 0.0)
    assert g.step(0.0)[:2] == (0.0, 0.0)
    g.set_enabled(True, 1.0)
    assert g.step(1.0)[2] == "stale"


def test_enable_must_keep_arriving():
    g = gate()
    ramp(g, 0, 0.5)
    g.nav_cmd(0.5, 0, 6.0)
    assert g.step(6.0)[2] == "nav"
    g.nav_cmd(0.5, 0, 7.0)  # mission silent since t=4.95
    assert g.step(7.0) == (0.0, 0.0, "disabled")


def test_speed_caps_indoor_outdoor_and_phone_loss():
    assert abs(ramp(gate(), 0, 2.0)[0] - 0.6) < 1e-9
    g = gate()
    assert abs(ramp(g, 0, 2.0, outdoor=True)[0] - 1.0) < 1e-9
    assert abs(ramp(g, 10, 2.0, outdoor=True, phone=False)[0] - 0.6) < 1e-9  # phone bridge silent
    g2 = gate()
    g2.set_phone_alive(False, 0.0)
    assert abs(ramp(g2, 0, 2.0, outdoor=True, phone=False)[0] - 0.6) < 1e-9


def test_non_finite_commands_are_dropped():
    g = gate()
    for t in (0.0, 0.05, 0.1):
        out = tick(g, t, float("nan"), float("nan"))
    assert out == (0.0, 0.0, "stale")
    g.manual_cmd(float("inf"), 0.0, 0.2)
    assert g.step(0.2)[2] == "stale"
    g.person(float("nan"), 0.3)
    assert g.speed_cap(0.3) == 0.0


def test_turn_rate_cap_slows_the_car_on_the_same_curve():
    lin, ang, _ = ramp(gate(), 0, 0.3, 3.0)  # a 0.1 m radius at 3 rad/s
    assert abs(ang - math.radians(30)) < 1e-9
    assert ang / lin == pytest.approx(10.0)  # same curvature: the Ackermann car follows the same arc


def test_speed_caps_keep_the_curve():
    lin, ang, _ = bumper_ramp(gate(), 0, 0.6, 0.6, ang=0.3)  # 2 m radius, something 0.6 m ahead
    assert lin < 0.6 and ang / lin == pytest.approx(0.5)


def test_acceleration_limited_braking_not():
    g = gate()
    tick(g, 0.0, 0.6)
    lin, _, _ = tick(g, 0.1, 0.6)
    assert lin <= 0.1 + 1e-9  # 0.05 on the first step, +0.5 m/s^2 * 0.1 s
    ramp(g, 0.1, 0.6)
    assert tick(g, 6.0, 0.0)[0] == 0.0


def test_reversal_stops_first():
    g = gate()
    ramp(g, 0, 0.6)
    assert tick(g, 5.0, -0.2)[0] == 0.0
    lin = tick(g, 5.05, -0.2)[0]
    assert -0.2 <= lin < 0.0


def test_person_stop_and_slow():
    g = gate()
    ramp(g, 0, 0.6)
    g.person(0.8, 5.0)
    assert tick(g, 5.0, 0.6, 0.2) == (0.0, 0.0, "person")
    g.person(2.0, 5.1)
    assert 0.25 < g.speed_cap(5.1) < 0.6


def test_silent_person_detector_means_creep():
    g = VelocityGate(GateParams())
    assert g.speed_cap(10.0) == GateParams().person_slow_speed


def test_manual_beats_recovery_beats_nav_and_recovery_expires():
    g = gate()
    g.set_enabled(True, 0.0)
    g.nav_cmd(0.5, 0.0, 0.0)
    g.set_recovery(True, 0.0)
    assert g.step(0.0)[2] == "recovery"
    g.manual_cmd(0.2, 0.0, 0.0)
    assert g.step(0.0)[2] == "manual"
    assert g.step(1.0)[2] == "recovery"
    g.set_enabled(True, 100.0)
    assert g.step(100.0)[2] == "stale"  # the mission stopped re-sending recovery


def test_reverse_limited():
    assert abs(ramp(gate(), 0, -1.0)[0] + 0.2) < 1e-9


def bumper_ramp(g, t0, lin, dist, ang=0.0, secs=5.0):
    t, out = t0, None
    while t < t0 + secs:
        g.bumper(dist, t)
        out = tick(g, t, lin, ang)
        t += 0.05
    return out


def test_bumper_stops_forward_only():
    g = gate()
    lin, ang, why = bumper_ramp(g, 0, 0.5, 0.3, ang=0.2)
    assert lin == 0.0 and ang == 0.0 and why == "bumper"  # an Ackermann car cannot turn away without moving
    lin, _, _ = bumper_ramp(g, 10, -0.1, 0.3)
    assert lin == -0.1  # and back off


def test_bumper_slows_in_between():
    g = gate()
    lin, _, _ = bumper_ramp(g, 0, 0.6, 0.675)  # halfway between stop (0.35) and slow (1.0)
    assert 0.15 < lin < 0.6
    lin, _, _ = bumper_ramp(g, 10, 0.6, math.inf)
    assert lin == 0.6


def test_bumper_optional_unless_required():
    g = gate()
    assert ramp(g, 0, 0.5)[0] == 0.5  # no sensor fitted: no limit
    g = gate(require_bumper=True)
    assert ramp(g, 0, 0.5)[0] == GateParams().bumper_slow_speed  # required but silent: creep


def test_outdoor_flag_must_keep_arriving():
    g = gate()
    assert ramp(g, 0, 1.0, outdoor=True)[0] == 1.0
    g.set_enabled(True, 10.0), g.set_phone_alive(True, 10.0), g.nav_cmd(1.0, 0, 10.0)
    assert g.step(10.0)[0] == GateParams().indoor_max  # outdoor flag 5 s old: indoor cap


def drive(g, t0, lin, secs, speed=None, accel=None):
    """Ticks at 20 Hz with the phone's accelerometer at 100 Hz and odometry at 20 Hz."""
    t, out = t0, None
    while t < t0 + secs:
        for k in range(5):
            ax, ay = accel(t + k * 0.01) if accel else (0.3, 0.1)  # a slight tilt, gravity leaking in
            g.accel(ax, ay, t + k * 0.01)
        if speed is not None:
            g.odom_speed(speed(t), t)
        out = tick(g, t, lin)
        t += 0.05
    return out, t


def test_jolt_while_moving_blocks_forward_then_releases():
    g = gate()
    out, t = drive(g, 0, 0.5, 3.0, speed=lambda t: min(0.5, t))
    assert out[0] > 0.4 and out[2] == "nav"
    hit = t + 0.2
    out, t = drive(g, t, 0.5, 0.5, speed=lambda _: 0.0, accel=lambda tt: (-9.0, 0.5) if hit <= tt < hit + 0.03 else (0.3, 0.1))
    assert out[:3] == (0.0, 0.0, "collision") and g.s.collisions == 1
    back = [tick(g, t + k * 0.05, -0.1) for k in range(4)]
    assert back[-1][0] < 0 and back[-1][2] == "nav"  # backing away is allowed (ramped like any start)
    out, _ = drive(g, t + 3.5, 0.5, 1.0, speed=lambda tt: 0.3)
    assert out[2] == "nav" and out[0] > 0  # released after collision_hold


def test_ordinary_driving_and_braking_are_not_collisions():
    g = gate()
    # speeding up, cruising, a gentle brake (2.5 m/s^2) and floor joints (vertical: not seen here)
    drive(g, 0, 0.6, 4.0, speed=lambda t: min(0.6, 0.5 * t), accel=lambda t: (0.5 + 0.2 * math.sin(40 * t), 0.1))
    drive(g, 4.0, 0.0, 1.0, speed=lambda t: 0.0, accel=lambda t: (-2.5, 0.1) if t < 4.3 else (0.3, 0.1))
    assert g.s.collisions == 0


def test_jolt_while_standing_still_is_ignored():
    g = gate()
    drive(g, 0, 0.0, 1.0, accel=lambda t: (8.0, 0.0) if 0.5 <= t < 0.52 else (0.3, 0.1))  # someone knocks the car
    assert g.s.collisions == 0


def test_pushing_against_something_is_a_stall():
    g = gate()
    out, t = drive(g, 0, 0.4, 3.0, speed=lambda _: 0.0)  # told to go, never moves
    assert out[2] == "collision" and g.s.collision_kind == "stalled"


def test_slow_start_is_not_a_stall():
    g = gate()
    out, _ = drive(g, 0, 0.4, 3.0, speed=lambda t: min(0.4, 0.4 * t))
    assert g.s.collisions == 0 and out[2] == "nav"


def test_no_odometry_means_no_stall_check():
    g = gate()
    out, _ = drive(g, 0, 0.4, 3.0)  # no speed readings at all
    assert g.s.collisions == 0 and out[0] > 0
