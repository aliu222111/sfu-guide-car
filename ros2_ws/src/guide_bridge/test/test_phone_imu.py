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


def test_clock_offset_takes_minimum_then_freezes():
    c = ClockOffset(n=3)
    c.update(100.0, 1000.05)
    c.update(101.0, 1001.02)
    c.update(102.0, 1002.08)
    assert abs(c.offset - 900.02) < 1e-9
    c.update(103.0, 1003.0)  # past n: ignored
    assert abs(c.to_local(103.0) - 1003.02) < 1e-9


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
