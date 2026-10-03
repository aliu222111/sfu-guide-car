"""One-camera odometry node: floor-plane visual odometry on ``/vo`` (spec section 18).

Drop-in for RTAB-Map's stereo odometry when only one camera works: same topic,
same message, same "lost" convention (pose covariance 9999), so the EKFs, the
localization manager and the velocity gate are unchanged. The camera's height
and tilt come from TF (the URDF), and must be measured on the car: a 1 cm
height error is a 2 % distance error.
"""

from __future__ import annotations

import math

import cv2
from nav_msgs.msg import Odometry
from rclpy.time import Time

from .base_node import CameraNode, spin
from .ground_odometry import GroundOdometry, GroundOdomParams, compose

LOST_COVARIANCE = 9999.0


class MonoOdometryNode(CameraNode):
    def __init__(self):
        super().__init__("mono_odometry", default_rate=10.0)
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.params = GroundOdomParams(
            min_range=self.declare_parameter("min_range", 0.4).value,
            max_range=self.declare_parameter("max_range", 4.0).value,
        )
        self.pub = self.create_publisher(Odometry, "vo", 10)
        self.odo = None
        self.pose = (0.0, 0.0, 0.0)
        self.last_stamp = None

    def process(self, bgr, header):
        if self.K is None:
            return
        cam = self.camera_pose()
        if cam is None:
            return
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.odo is None:
            self.odo = GroundOdometry(self.K, cam[0], cam[1], gray.shape, self.params)
        step = self.odo.update(gray)
        stamp = Time.from_msg(header.stamp).nanoseconds * 1e-9
        dt = stamp - self.last_stamp if self.last_stamp is not None else 0.0
        self.last_stamp = stamp
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = header.stamp, self.odom_frame, self.base_frame
        if step is None:
            if dt == 0.0:
                return  # first frame
            # Lost, as rtabmap reports it: the localization manager stops the car.
            m.pose.covariance[0] = m.pose.covariance[7] = m.pose.covariance[35] = LOST_COVARIANCE
            m.twist.covariance[0] = m.twist.covariance[35] = LOST_COVARIANCE
            self.pub.publish(m)
            return
        self.pose = compose(self.pose, step)
        x, y, yaw = self.pose
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # Fewer inliers, less trust. 0.02 m per frame at 100 inliers.
        sigma = 0.02 * math.sqrt(100.0 / max(step.inliers, 1))
        m.pose.covariance[0] = m.pose.covariance[7] = sigma**2
        m.pose.covariance[35] = (sigma / 2) ** 2
        if dt > 0:
            m.twist.twist.linear.x = step.dx / dt
            m.twist.twist.linear.y = step.dy / dt
            m.twist.twist.angular.z = step.dyaw / dt
            m.twist.covariance[0] = m.twist.covariance[7] = (sigma / dt) ** 2
            m.twist.covariance[35] = (sigma / 2 / dt) ** 2
        self.pub.publish(m)


def main():
    spin(MonoOdometryNode)
