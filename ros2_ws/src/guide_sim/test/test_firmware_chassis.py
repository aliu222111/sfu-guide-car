"""The real firmware, compiled for the PC, working the simulated hand controller and car."""

import math
from pathlib import Path

import pytest

from guide_bridge.ackermann import Ackermann
from guide_bridge.protocol import format_drive
from guide_bridge.speed import SpeedTrim
from guide_sim.chassis import Chassis, ChassisParams
from guide_sim.firmware import Firmware

INO = Path(__file__).resolve().parents[2] / "guide_bridge" / "firmware" / "guide_motor" / "guide_motor.ino"
STEER, THROTTLE = 9, 10


@pytest.fixture
def fw():
    f = Firmware(INO)
    f.start()
    yield f
    f.close()


def volts(fw, pin):
    return fw.pwm(pin) / 255 * 3.3


def drive(fw, ch, v, w, seconds, t0=0.0, rate=20.0, car=None):
    """The serial bridge's Ackermann maths at ``rate``, the firmware and chassis at 1 kHz."""
    car = car or Ackermann()
    t = t0
    next_cmd = t0
    while t < t0 + seconds:
        if t >= next_cmd:
            fw.serial_in(format_drive(*car.command(v, w)))
            next_cmd += 1.0 / rate
        t += 0.001
        fw.run_until(t)
        ch.step(0.001, *ch.targets(fw))
    return t


def test_starts_centred(fw):
    fw.run_until(0.05)
    assert volts(fw, STEER) == pytest.approx(1.65, abs=0.01) and volts(fw, THROTTLE) == pytest.approx(1.65, abs=0.01)


def test_wheel_and_trigger_voltages(fw):
    fw.serial_in(b"C 1.0 1.0\n")
    fw.run_until(0.01)
    assert volts(fw, STEER) == pytest.approx(0.35, abs=0.01)  # full left
    assert volts(fw, THROTTLE) == pytest.approx(2.95, abs=0.01)  # full trigger
    fw.serial_in(b"C -1.0 -0.5\n")
    fw.run_until(0.02)
    assert volts(fw, STEER) == pytest.approx(2.95, abs=0.01)  # full right mirrors left
    assert volts(fw, THROTTLE) == pytest.approx(1.0, abs=0.01)  # half reverse


def test_straight_line(fw):
    ch = Chassis()
    drive(fw, ch, 0.6, 0.0, 3.0)
    assert ch.s.x > 1.0
    # Not yet calibrated: a little slower than asked and drifting with the steering trim.
    assert 0.4 < ch.s.v < 0.6 and abs(ch.s.y) < 0.3


def test_turning_left_is_counter_clockwise(fw):
    ch = Chassis()
    drive(fw, ch, 0.6, 0.6, 4.0)  # radius 1 m
    assert ch.s.yaw > 1.0 and ch.s.y > 0.3


def test_cannot_turn_on_the_spot(fw):
    ch = Chassis()
    drive(fw, ch, 0.0, 0.5, 2.0)
    assert ch.s.yaw == 0.0 and ch.s.x == 0.0  # an Ackermann car does not pivot
    assert ch.s.steer > 0.4  # but the wheels point left, ready to go


def test_tight_turn_is_limited_by_full_lock(fw):
    ch = Chassis()
    drive(fw, ch, 0.5, 2.0, 3.0)  # asks for a 0.25 m radius
    p = ChassisParams()
    assert ch.s.v / ch.s.w == pytest.approx(p.wheelbase / math.tan((1 - p.steer_offset) * p.max_steer) * p.understeer, rel=0.05)


def test_watchdog_centres(fw):
    ch = Chassis()
    t = drive(fw, ch, 0.5, 0.3, 1.0)
    assert volts(fw, THROTTLE) > 1.7
    fw.run_until(t + 0.25)
    assert volts(fw, THROTTLE) > 1.7  # 250 ms without a command: still driving
    fw.run_until(t + 0.4)
    assert volts(fw, THROTTLE) == pytest.approx(1.65, abs=0.01) and volts(fw, STEER) == pytest.approx(1.65, abs=0.01)


def test_uncalibrated_slow_speed_stalls_and_the_speed_trim_fixes_it(fw):
    # The gate's slow-downs near people ask for 0.15 m/s. The car's real deadband is
    # wider than guide.yaml's throttle_start, so open loop it barely moves.
    ch = Chassis()
    drive(fw, ch, 0.15, 0.0, 3.0)
    assert ch.s.v < 0.05
    trim, t = SpeedTrim(), 3.0
    for _ in range(160):  # 8 s of the serial bridge at 20 Hz, measuring the car's speed
        lin, ang = trim.command(0.15, 0.0, ch.s.v, 0.1, 0.05)
        t = drive(fw, ch, lin, ang, 0.05, t0=t)
    assert ch.s.v == pytest.approx(0.15, abs=0.03)


def test_garbage_lines_ignored(fw):
    fw.serial_in(b"C nonsense\nhello\n" + b"X" * 200 + b"\nC 0.0 0.3\n")
    fw.run_until(0.01)
    assert volts(fw, THROTTLE) > 1.9


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


def test_collision_counted():
    ch = Chassis()
    ch.place(0, 0, 0)
    wall = lambda x, y, yaw: x > 0.2
    for _ in range(2000):
        ch.step(0.001, 0.5, 0.0, wall)
    assert ch.s.collisions == 1 and ch.s.x <= 0.2
