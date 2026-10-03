"""The real firmware, compiled for the PC, driving the simulated Ackermann chassis."""

import math
from pathlib import Path

import pytest

from guide_sim.chassis import Chassis, ChassisParams
from guide_sim.firmware import Firmware

INO = Path(__file__).resolve().parents[2] / "guide_bridge" / "firmware" / "guide_motor" / "guide_motor.ino"
STEER, ESC = 9, 10


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


def test_arms_at_neutral(fw):
    fw.run_until(0.05)
    assert fw.servo_us(ESC) == 1500 and fw.servo_us(STEER) == 1500


def test_straight_line(fw):
    ch = Chassis()
    drive(fw, ch, 0.6, 0.0, 3.0)
    assert ch.s.x > 1.0 and abs(ch.s.y) < 1e-6
    # Open loop and not yet calibrated: slower than asked (the bridge's speed trim closes the loop).
    assert 0.4 < ch.s.v < 0.6


def test_reverse(fw):
    ch = Chassis()
    drive(fw, ch, -0.4, 0.0, 2.0)
    assert ch.s.x < -0.3


def test_turning_left_is_counter_clockwise_and_follows_the_bicycle_model(fw):
    ch = Chassis()
    drive(fw, ch, 0.6, 0.6, 4.0)  # radius 1 m: steer atan(0.48), inside full lock
    assert ch.s.yaw > 1.0 and ch.s.y > 0.3
    # The firmware's wheelbase matches; understeer and the speed error make it a little wide.
    assert ch.s.w / ch.s.v == pytest.approx(1.0, rel=0.25)


def test_cannot_turn_on_the_spot(fw):
    ch = Chassis()
    drive(fw, ch, 0.0, 0.5, 2.0)
    assert ch.s.yaw == 0.0 and ch.s.x == 0.0  # an Ackermann car does not pivot
    assert ch.s.steer > 0.4  # but the wheels point left, ready to go


def test_tight_turn_is_limited_by_full_lock(fw):
    ch = Chassis()
    drive(fw, ch, 0.5, 2.0, 2.0)  # asks for a 0.25 m radius
    assert fw.servo_us(STEER) == 1500 - 400  # full lock (a shorter pulse steers left)
    radius = ch.s.v / ch.s.w
    assert radius == pytest.approx(0.48 / math.tan(ChassisParams().max_steer) * 1.1, rel=0.05)


def test_watchdog_goes_to_neutral(fw):
    ch = Chassis()
    t = drive(fw, ch, 0.5, 0.0, 1.0)
    assert fw.servo_us(ESC) > 1500
    fw.run_until(t + 0.4)
    assert fw.servo_us(ESC) > 1500  # 400 ms without a command: still driving
    fw.run_until(t + 0.6)
    assert fw.servo_us(ESC) == 1500  # past 500 ms: neutral


def test_throttle_is_capped(fw):
    fw.serial_in(b"V 5.0 0.0\n")  # far beyond anything the laptop sends
    fw.run_until(0.05)
    assert fw.servo_us(ESC) == 1500 + 180


def test_uncalibrated_slow_speed_stalls_and_the_speed_trim_fixes_it(fw):
    # The gate's slow-downs near people ask for 0.15 m/s. The ESC's real deadband is
    # wider than the firmware's ESC_START_US, so open loop the car barely moves.
    from guide_bridge.speed import SpeedTrim

    ch = Chassis()
    drive(fw, ch, 0.15, 0.0, 3.0)
    assert ch.s.v < 0.05
    trim, t = SpeedTrim(), 3.0
    for _ in range(160):  # 8 s of the serial bridge at 20 Hz, measuring the car's speed
        lin, ang = trim.command(0.15, 0.0, ch.s.v, 0.1, 0.05)
        t = drive(fw, ch, lin, ang, 0.05, t0=t)
    assert ch.s.v == pytest.approx(0.15, abs=0.03)


def test_brake_before_reverse():
    fw = Firmware(INO, defines=("HAVE_ULTRASONIC", "ESC_BRAKE_BEFORE_REVERSE"))
    fw.start()
    try:
        fw.serial_in(b"V 0.5 0.0\n")
        fw.run_until(0.2)
        assert fw.servo_us(ESC) > 1500
        fw.serial_in(b"V -0.5 0.0\n")
        fw.run_until(0.25)
        assert fw.servo_us(ESC) < 1500  # brake
        fw.serial_in(b"V -0.5 0.0\n")
        fw.run_until(0.45)
        assert fw.servo_us(ESC) == 1500  # back to neutral
        fw.serial_in(b"V -0.5 0.0\n")
        fw.run_until(0.6)
        assert fw.servo_us(ESC) < 1500 - 60  # reversing
    finally:
        fw.close()


def test_garbage_lines_ignored(fw):
    fw.serial_in(b"V nonsense\nhello\n" + b"X" * 200 + b"\nV 0.3 0.0\n")
    fw.run_until(0.01)
    assert fw.servo_us(ESC) > 1500


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
