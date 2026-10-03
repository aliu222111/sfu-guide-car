"""The phone bridge node itself: constructs, listens, and turns a push into an Imu message."""

import json
import time
import urllib.request

import pytest

rclpy = pytest.importorskip("rclpy")


def test_push_becomes_imu():
    from sensor_msgs.msg import Imu

    from guide_bridge.phone_imu_node import PhoneImuNode

    rclpy.init(args=["--ros-args", "-p", "port:=0"])
    try:
        node = PhoneImuNode()  # once crashed here: a method named "handle" hid rclpy's Node.handle
        port = node.server.server_address[1]
        got = []
        sub_node = rclpy.create_node("listener")
        sub_node.create_subscription(Imu, "phone/imu", got.append, 10)
        t = int(1_790_000_000 * 1e9)
        body = json.dumps({"payload": [
            {"name": "gyroscope", "time": t, "values": {"x": 0.0, "y": 0.0, "z": 0.3}},
            {"name": "totalacceleration", "time": t, "values": {"x": 0.0, "y": 0.0, "z": 9.81}},
        ]}).encode()
        urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{port}/data", data=body), timeout=2).read()
        end = time.time() + 5
        while not got and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(sub_node, timeout_sec=0.05)
        assert got and got[0].angular_velocity.z == pytest.approx(0.3)
        node.destroy_node()
        sub_node.destroy_node()
    finally:
        rclpy.try_shutdown()
