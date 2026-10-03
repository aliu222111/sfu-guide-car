"""The real firmware, compiled for the PC, driving the simulated chassis."""

from pathlib import Path

import pytest

from guide_sim.chassis import Chassis, ChassisParams, track_target
from guide_sim.firmware import Firmware

INO = Path(__file__).resolve().parents[2] / "guide_bridge" / "firmware" / "guide_motor" / "guide_motor.ino"


@pytest.fixture
def fw():
    f = Firmware(INO)
    f.start()
    yield f
    f.close()


def drive(fw, ch, v, w, seconds, t0=0.0, rate=20.0):
    """Send V lines at ``rate`` like the serial bridge and step the chassis at 1 kHz."""
    t = t0
    next_cmd = t0
    while t < t0 + seconds:
        if t >= next_cmd:
            fw.serial_in(f"V {v:.3f} {w:.3f}\n".encode())
            next_cmd += 1.0 / rate
        t += 0.001
        fw.run_until(t)
        ch.step(0.001, *ch.targets(fw))
    return t


def test_straight_line_both_tracks_forward(fw):
    ch = Chassis()
    drive(fw, ch, 0.4, 0.0, 3.0)
    assert ch.s.x > 0.8 and abs(ch.s.y) < 0.01 and abs(ch.s.yaw) < 1e-6
    assert ch.s.v == pytest.approx(ch.s.v_left)  # straight: both tracks equal


def test_turning_left_is_counter_clockwise(fw):
    ch = Chassis()
    drive(fw, ch, 0.0, 0.5, 3.0)
    assert ch.s.yaw > 0.3  # left = counter-clockwise: the right motor's mirrored mounting is handled
    assert abs(ch.s.x) < 1e-9 and abs(ch.s.y) < 1e-9  # a pivot in place


def test_fast_turns_slip(fw):
    ch = Chassis()
    drive(fw, ch, 0.0, 1.4, 3.0)  # past the gate's cap, so the PWM floor is a small part of the duty
    assert 0.5 < ch.s.w < 1.4  # tracks slip: slower than asked


def test_watchdog_stops_motors(fw):
    ch = Chassis()
    t = drive(fw, ch, 0.4, 0.0, 1.0)
    pins = (5, 6)
    assert all(fw.pwm(p) > 0 for p in pins)
    fw.run_until(t + 0.4)
    assert all(fw.pwm(p) > 0 for p in pins)  # 400 ms without a command: still driving
    fw.run_until(t + 0.6)
    assert all(fw.pwm(p) == 0 for p in pins)  # past 500 ms: stopped


def test_garbage_lines_ignored(fw):
    fw.serial_in(b"V nonsense\nhello\n" + b"X" * 200 + b"\nV 0.3 0.0\n")
    fw.run_until(0.01)
    assert fw.pwm(5) > 0 and fw.pwm(6) > 0


def test_button_reported(fw):
    fw.run_until(0.01)
    fw.serial_out()
    fw.set_input(2, False)  # pressed: to ground
    fw.run_until(0.05)
    assert b"B 1\r\n" in fw.serial_out()
    fw.set_input(2, True)
    fw.run_until(0.1)
    assert b"B 0\r\n" in fw.serial_out()


def test_ultrasonic_reported(fw):
    fw.set_echo_us(87 * 58)
    fw.run_until(0.35)
    assert b"U 87" in fw.serial_out().split(b"\r\n")
    fw.set_echo_us(38000)  # HC-SR04 with nothing in range
    fw.run_until(0.6)
    assert b"U 0" in fw.serial_out().split(b"\r\n")
    fw.set_echo_us(0)  # no pulse at all: unplugged
    fw.run_until(0.9)
    assert b"U -1" in fw.serial_out().split(b"\r\n")


def test_slow_straight_driving_is_not_inflated(fw):
    # The bumper slow-down: 0.15 m/s. A pivot floor applied here gave about 0.2.
    ch = Chassis()
    drive(fw, ch, 0.15, 0.0, 3.0)
    assert ch.s.v == pytest.approx(0.15, abs=0.02)


def test_min_pwm_and_stiction():
    p = ChassisParams()
    assert track_target(40, 1, 1, p) == 0.0  # under stiction
    assert track_target(255, 1, 1, p) == pytest.approx(p.top_speed)
    assert track_target(255, 0, 1, p) == pytest.approx(-p.top_speed)


def test_slow_pivot_stalls_without_min_pivot_rate(fw):
    # 0.1 rad/s asks for 0.015 m/s per side. MIN_PWM_PIVOT gets the tracks turning, but
    # not hard enough to drag the car round: the bridge's min_pivot_rate exists for this.
    ch = Chassis()
    drive(fw, ch, 0.0, 0.1, 2.0)
    assert ch.s.yaw == 0.0
    drive(fw, ch, 0.0, 0.45, 2.0, t0=2.0)  # Nav2's rotate-to-heading rate does turn it
    assert ch.s.yaw > 0.3


def test_min_pwm_set_too_high_overshoots(fw):
    # Tracks that pivot easily: MIN_PWM_PIVOT 65 makes every slow pivot a fast one.
    ch = Chassis(ChassisParams(stiction_duty=0.15, pivot_breakaway=0.05))
    drive(fw, ch, 0.0, 0.1, 2.0)
    assert ch.s.w > 0.2


def test_min_pwm_set_too_low_stalls_tracks(fw):
    ch = Chassis(ChassisParams(stiction_duty=0.30))  # stiffer floor: PWM 65 does not move a track
    drive(fw, ch, 0.0, 0.1, 2.0)
    assert ch.s.v_left == 0.0 and ch.s.v_right == 0.0


def test_collision_counted():
    ch = Chassis()
    ch.place(0, 0, 0)
    wall = lambda x, y, yaw: x > 0.2
    for _ in range(2000):
        ch.step(0.001, 0.5, 0.5, wall)
    assert ch.s.collisions == 1 and ch.s.x <= 0.2
