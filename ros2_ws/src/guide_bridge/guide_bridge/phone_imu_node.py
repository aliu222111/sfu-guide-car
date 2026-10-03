"""Receive Sensor Logger HTTP pushes from the phone strapped to the car (spec section 3).

In Sensor Logger: Settings > Data Streaming > HTTP Push, URL
``http://<laptop>:8765/data``, sensors: accelerometer (or total acceleration),
gravity, gyroscope, orientation, magnetometer, location; 100 Hz.
"""

from __future__ import annotations

import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus
from std_msgs.msg import Bool

from .phone_imu import ClockOffset, Liveness, parse_payload

MAX_BODY = 2_000_000


class PhoneImuNode(Node):
    def __init__(self):
        super().__init__("phone_imu")
        self.port = self.declare_parameter("port", 8765).value
        self.frame = self.declare_parameter("frame_id", "phone_link").value
        self.accel_scale = self.declare_parameter("accel_scale", 1.0).value
        self.yaw_offset = self.declare_parameter("world_yaw_offset", 0.0).value
        self.gyro_var = self.declare_parameter("gyro_variance", 1e-4).value
        self.accel_var = self.declare_parameter("accel_variance", 1e-2).value
        self.orient_var = self.declare_parameter("orientation_variance", 3e-3).value
        self.imu_pub = self.create_publisher(Imu, "phone/imu", 50)
        self.mag_pub = self.create_publisher(MagneticField, "phone/mag", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "phone/fix", 10)
        self.alive_pub = self.create_publisher(Bool, "phone/alive", 10)
        self.clock_offset = ClockOffset()
        self.live = Liveness(self.declare_parameter("timeout", 1.0).value)
        self.inbox: queue.Queue = queue.Queue(maxsize=200)
        self.create_timer(0.005, self.drain)
        self.create_timer(0.2, self.publish_alive)
        self._start_server()

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                if n <= 0 or n > MAX_BODY:
                    self.send_response(413 if n > MAX_BODY else 400)
                    self.end_headers()
                    return
                body = self.rfile.read(n)
                recv = node.now()
                try:
                    doc = json.loads(body)
                except ValueError:
                    self.send_response(400)
                    self.end_headers()
                    return
                try:
                    node.inbox.put_nowait((recv, doc))
                except queue.Full:
                    pass  # drop rather than fall behind
                self.send_response(200)
                self.end_headers()

            def log_message(self, *a):
                pass

        self.server = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f"listening for Sensor Logger on port {self.port}")

    def stamp(self, phone_t: float):
        return Time(seconds=self.clock_offset.to_local(phone_t)).to_msg()

    def drain(self):
        while True:
            try:
                recv, doc = self.inbox.get_nowait()
            except queue.Empty:
                return
            try:
                self.handle(recv, doc)
            except Exception as e:  # never let one bad push take the IMU down
                self.get_logger().warn(f"dropped a phone push: {type(e).__name__}: {e}", throttle_duration_sec=5.0)

    def handle(self, recv, doc):
        imus, mags, fixes = parse_payload(doc, self.accel_scale, self.yaw_offset)
        if not (imus or mags or fixes):
            return
        self.live.seen(recv)
        newest = max(s.t for s in (*imus, *mags, *fixes))
        self.clock_offset.update(newest, recv)
        for s in imus:
            self.imu_pub.publish(self.imu_msg(s))
        for s in mags:
            m = MagneticField()
            m.header.stamp, m.header.frame_id = self.stamp(s.t), self.frame
            m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = s.field
            self.mag_pub.publish(m)
        for s in fixes:
            f = NavSatFix()
            f.header.stamp, f.header.frame_id = self.stamp(s.t), self.frame
            f.status.status = NavSatStatus.STATUS_FIX
            f.status.service = NavSatStatus.SERVICE_GPS
            f.latitude, f.longitude, f.altitude = s.lat, s.lon, s.alt
            h, v = s.horizontal_accuracy**2, s.vertical_accuracy**2
            f.position_covariance = [h, 0.0, 0.0, 0.0, h, 0.0, 0.0, 0.0, v]
            f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(f)

    def imu_msg(self, s) -> Imu:
        m = Imu()
        m.header.stamp, m.header.frame_id = self.stamp(s.t), self.frame
        if s.orientation is not None:
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = s.orientation
            m.orientation_covariance = [self.orient_var, 0.0, 0.0, 0.0, self.orient_var, 0.0, 0.0, 0.0, self.orient_var]
        else:
            m.orientation_covariance[0] = -1.0
        if s.gyro is not None:
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = s.gyro
            m.angular_velocity_covariance = [self.gyro_var, 0.0, 0.0, 0.0, self.gyro_var, 0.0, 0.0, 0.0, self.gyro_var]
        else:
            m.angular_velocity_covariance[0] = -1.0
        if s.accel is not None:
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = s.accel
            m.linear_acceleration_covariance = [self.accel_var, 0.0, 0.0, 0.0, self.accel_var, 0.0, 0.0, 0.0, self.accel_var]
        else:
            m.linear_acceleration_covariance[0] = -1.0
        return m

    def publish_alive(self):
        self.alive_pub.publish(Bool(data=self.live.alive(self.now())))

    def shutdown(self):
        self.server.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PhoneImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
