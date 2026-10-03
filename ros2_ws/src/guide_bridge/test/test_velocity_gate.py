import math

from guide_bridge.velocity_gate import GateParams, VelocityGate


def gate(**kw):
    g = VelocityGate(GateParams(require_person_detector=False, **kw))
    g.set_enabled(True)
    return g


def ramp(g, t0, lin, ang=0.0, secs=5.0, outdoor=False):
    g.set_outdoor(outdoor)
    t = t0
    out = None
    while t < t0 + secs:
        g.nav_cmd(lin, ang, t)
        out = g.step(t)
        t += 0.05
    return out


def test_disabled_and_stale_stop():
    g = VelocityGate()
    g.nav_cmd(0.5, 0, 0.0)
    assert g.step(0.0)[:2] == (0.0, 0.0)
    g.set_enabled(True)
    assert g.step(1.0)[2] == "stale"


def test_speed_caps_indoor_outdoor_and_phone_loss():
    g = gate()
    assert abs(ramp(g, 0, 2.0)[0] - 0.6) < 1e-9
    g = gate()
    assert abs(ramp(g, 0, 2.0, outdoor=True)[0] - 1.0) < 1e-9
    g.set_phone_alive(False)
    lin = ramp(g, 10, 2.0, outdoor=True)[0]
    assert abs(lin - 0.6) < 1e-9


def test_turn_rate_cap():
    g = gate()
    _, ang, _ = ramp(g, 0, 0.3, 3.0)
    assert abs(ang - math.radians(30)) < 1e-9


def test_acceleration_limited_braking_not():
    g = gate()
    g.nav_cmd(0.6, 0, 0.0)
    g.step(0.0)
    g.nav_cmd(0.6, 0, 0.1)
    lin, _, _ = g.step(0.1)
    assert lin <= 0.1 + 1e-9  # 0.05 on the first step, +0.5 m/s^2 * 0.1 s
    ramp(g, 0.1, 0.6)
    g.nav_cmd(0.0, 0, 6.0)
    assert g.step(6.0)[0] == 0.0


def test_person_stop_and_slow():
    g = gate()
    ramp(g, 0, 0.6)
    g.person(0.8, 5.0)
    g.nav_cmd(0.6, 0.2, 5.0)
    assert g.step(5.0) == (0.0, 0.0, "person")
    g.person(2.0, 5.1)
    g.nav_cmd(0.6, 0, 5.1)
    cap = g.speed_cap(5.1)
    assert 0.25 < cap < 0.6


def test_silent_person_detector_means_creep():
    g = VelocityGate(GateParams())
    g.set_enabled(True)
    assert g.speed_cap(10.0) == GateParams().person_slow_speed


def test_manual_beats_recovery_beats_nav():
    g = gate()
    g.nav_cmd(0.5, 0.0, 0.0)
    g.set_recovery(True)
    assert g.step(0.0)[2] == "recovery"
    g.manual_cmd(0.2, 0.0, 0.0)
    assert g.step(0.0)[2] == "manual"
    assert g.step(1.0)[2] == "recovery"


def test_reverse_limited():
    g = gate()
    lin = ramp(g, 0, -1.0)[0]
    assert abs(lin + 0.2) < 1e-9
