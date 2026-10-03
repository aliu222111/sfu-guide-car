import pytest
import math

T = 1_790_000_000_000_000_000  # 2026, in ns

from guide_bridge.phone_imu import ClockOffset, Liveness, base_orientation, parse_payload, quat_yaw

S = math.sin(math.pi / 4)


def test_axes_mapping():
    doc = {"payload": [
        {"name": "gyroscope", "time": T, "values": {"x": 0.1, "y": 0.2, "z": 0.3}},
        {"name": "totalacceleration", "time": T, "values": {"x": 0.0, "y": 1.0, "z": 9.8}},
        {"name": "magnetometer", "time": T, "values": {"x": 20.0, "y": 0.0, "z": -40.0}},
    ]}
    imus, mags, fixes = parse_payload(doc)
    assert len(imus) == 1 and not fixes
    assert imus[0].gyro == (0.2, -0.1, 0.3)
    assert imus[0].accel == (1.0, -0.0, 9.8)  # phone pushed toward its top edge = car forward
    assert mags[0].field == pytest.approx((0.0, -20e-6, -40e-6))


def test_accelerometer_plus_gravity_when_no_total():
    doc = {"payload": [
        {"name": "accelerometer", "time": T, "values": {"x": 0.5, "y": 0.0, "z": 0.0}},
        {"name": "gravity", "time": T, "values": {"x": 0.0, "y": 0.0, "z": 9.81}},
    ]}
    imus, _, _ = parse_payload(doc)
    assert imus[0].accel == (0.0, -0.5, 9.81)


def test_orientation_phone_pointing_north_is_yaw_90():
    # Android rotation vector: identity = phone top edge pointing north (ENU y).
    q = base_orientation((0.0, 0.0, 0.0, 1.0))
    assert abs(quat_yaw(q) - math.pi / 2) < 1e-9
    # Phone rotated -90 deg about up: top edge east, car facing east, yaw 0.
    q = base_orientation((0.0, 0.0, -S, S))
    assert abs(quat_yaw(q)) < 1e-9


def test_location_and_bad_items_skipped():
    doc = {"payload": [
        {"name": "location", "time": T, "values": {"latitude": 49.28, "longitude": -122.91, "altitude": 300, "horizontalAccuracy": 4}},
        {"name": "gyroscope", "time": "x", "values": {}},
        {"name": "gyroscope", "time": T, "values": {"x": 1}},
    ]}
    imus, _, fixes = parse_payload(doc)
    assert imus == [] and fixes[0].horizontal_accuracy == 4.0


def test_clock_offset_is_a_sliding_minimum():
    c = ClockOffset(window=10.0)
    c.update(100.0, 1000.05)
    c.update(101.0, 1001.02)
    c.update(102.0, 1002.08)
    assert abs(c.offset - 900.02) < 1e-9
    assert abs(c.to_local(103.0) - 1003.02) < 1e-9


def test_clock_offset_forgets_a_startup_glitch_and_follows_drift():
    c = ClockOffset(window=10.0)
    c.update(100.0, 999.5)  # one push seen with a laptop clock not yet right
    assert abs(c.offset - 899.5) < 1e-9
    for k in range(1, 30):  # then 1 s pushes with 50 ms delay, the phone clock drifting 1 ms/s
        c.update(100.0 + k, 1000.05 + k + 0.001 * k)
    assert abs(c.offset - (900.05 + 0.001 * 20)) < 1e-6  # the oldest push still in the window


def test_liveness():
    l = Liveness(1.0)
    assert not l.alive(0.0)
    l.seen(5.0)
    assert l.alive(5.9) and not l.alive(6.1)


def test_malformed_pushes_are_ignored():
    for doc in ([1, 2], {"payload": None}, {"payload": [1]}, {"payload": [{"name": 5, "time": T}]},
                {"payload": [{"name": "gyroscope", "time": T, "values": {"x": float("nan"), "y": 0, "z": 0}}]},
                {"payload": [{"name": "gyroscope", "time": 10**30, "values": {"x": 0, "y": 0, "z": 0}}]},
                {"payload": [{"name": "location", "time": T, "values": {"latitude": 999, "longitude": 0}}]},
                {"payload": [{"name": "orientation", "time": T, "values": {"qx": 0, "qy": 0, "qz": 0, "qw": 0}}]}):
        assert parse_payload(doc) == ([], [], [])


def test_gyro_bias_learned_while_still_and_not_while_turning():
    from guide_bridge.phone_imu import GyroBias

    b = GyroBias(settle=2.0, tau=5.0)
    t = 0.0
    for _ in range(3000):  # 30 s standing still, the gyro reading 0.004 rad/s of bias
        t += 0.01
        b.odometry(t, 0.0, 0.0)
        out = b.gyro(t, 0.004, 0.0)
    assert b.bias == pytest.approx(0.004, abs=2e-4) and abs(out) < 2e-4
    for _ in range(500):  # turning: a real 0.3 rad/s is not bias
        t += 0.01
        b.odometry(t, 0.5, 0.3)
        out = b.gyro(t, 0.304, 0.0)
    assert b.bias == pytest.approx(0.004, abs=2e-4) and out == pytest.approx(0.3, abs=1e-3)


def test_gyro_bias_needs_fresh_odometry():
    from guide_bridge.phone_imu import GyroBias

    b = GyroBias(settle=1.0, tau=1.0)
    b.odometry(0.0, 0.0, 0.0)
    for k in range(500):
        b.gyro(0.01 * k, 0.02, odometry_age=5.0)  # the cameras stopped reporting: learn nothing
    assert b.bias == 0.0
