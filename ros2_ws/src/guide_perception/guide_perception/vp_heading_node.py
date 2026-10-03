"""Corridor vanishing-point heading (spec section 6).

Publishes ``guide/vp_heading`` (sensor_msgs/Imu with only yaw meaningful) for the
global EKF, when the mission has published the current corridor's direction
(``guide/corridor_axis``) in the last second.
"""

from __future__ import annotations

import math

import cv2
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_ros import TransformException

from .base_node import CameraNode, spin
from .vanishing import absolute_yaw, detect_lines, find_vanishing_point, oblique, relative_yaw, wrap


class VpHeadingNode(CameraNode):
    def __init__(self):
        super().__init__("vp_heading", default_rate=10.0)
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.max_rel = math.radians(self.declare_parameter("max_relative_deg", 30.0).value)
        self.max_jump = math.radians(self.declare_parameter("max_disagreement_deg", 20.0).value)
        self.sigma = math.radians(self.declare_parameter("yaw_sigma_deg", 3.0).value)
        self.camera_yaw = self.declare_parameter("camera_yaw_offset", 0.0).value
        self.axis = None
        self.axis_time = -math.inf
        self.create_subscription(Float64, "guide/corridor_axis", self.on_axis, 10)
        self.pub = self.create_publisher(Imu, "guide/vp_heading", 10)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_axis(self, msg):
        self.axis, self.axis_time = msg.data, self.now()

    def current_yaw(self):
        try:
            q = self.tf.lookup_transform(self.map_frame, self.base_frame, Time()).transform.rotation
            return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        except TransformException:
            return None

    def process(self, bgr, header):
        if self.K is None or self.axis is None or self.now() - self.axis_time > 1.0:
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        vp = find_vanishing_point(oblique(detect_lines(gray)))
        if vp is None:
            return
        rel = relative_yaw(vp.x, self.K[0, 0], self.K[0, 2]) + self.camera_yaw
        if abs(rel) > self.max_rel:
            return
        cur = self.current_yaw()
        if cur is None:
            return
        yaw = absolute_yaw(self.axis, rel, cur)
        if abs(wrap(yaw - cur)) > self.max_jump:
            return  # a side corridor or a poster, not this corridor
        m = Imu()
        m.header.stamp = header.stamp
        m.header.frame_id = self.base_frame
        m.orientation.z, m.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        big = 1e6
        m.orientation_covariance = [big, 0.0, 0.0, 0.0, big, 0.0, 0.0, 0.0, self.sigma**2]
        m.angular_velocity_covariance[0] = -1.0
        m.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(m)


def main():
    spin(VpHeadingNode)
