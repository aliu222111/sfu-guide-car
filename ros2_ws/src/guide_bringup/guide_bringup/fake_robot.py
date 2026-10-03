"""A kinematic stand-in for the car, for testing the software without hardware.

Integrates /cmd_vel (unicycle), publishes odom -> base_link and map -> odom
(identity) on TF, /vo odometry for the localization manager, empty sensor
clouds so Nav2's costmaps stay current, and "no person in view" for the
velocity gate, and the phone app's heartbeat. Indoors it sends one sign fix at start so the localization
manager considers the car placed on the floor.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, Float32, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.x = self.declare_parameter("x", 0.0).value
        self.y = self.declare_parameter("y", 0.0).value
        self.yaw = self.declare_parameter("yaw", 0.0).value
        self.indoor = self.declare_parameter("indoor", True).value
        self.fake_phone = self.declare_parameter("fake_phone", True).value
        self.v = self.w = 0.0
        self.odom = TransformBroadcaster(self)
        StaticTransformBroadcaster(self).sendTransform(self.tf("map", "odom", 0.0, 0.0, 0.0))
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.vo_pub = self.create_publisher(Odometry, "vo", 10)
        self.person_pub = self.create_publisher(Float32, "people/nearest", 10)
        self.clouds = [self.create_publisher(PointCloud2, t, 5) for t in ("stereo/points2", "floor/obstacles", "floor/free")]
        self.fix_pub = self.create_publisher(PoseWithCovarianceStamped, "guide/sign_fix", 10)
        self.heartbeat_pub = self.create_publisher(Empty, "guide/heartbeat", 10)
        self.dt = 0.02
        self.ticks = 0
        self.create_timer(self.dt, self.step)

    def tf(self, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return t

    def on_cmd(self, msg: Twist):
        self.v, self.w = msg.linear.x, msg.angular.z

    def step(self):
        self.ticks += 1
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        now = self.get_clock().now().to_msg()
        self.odom.sendTransform(self.tf("odom", "base_link", self.x, self.y, self.yaw))
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.pose.covariance[0] = o.pose.covariance[7] = 0.01
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.vo_pub.publish(o)
        if self.ticks % 5 == 0:
            self.person_pub.publish(Float32(data=math.inf))
            for pub in self.clouds:
                pub.publish(point_cloud2.create_cloud_xyz32(Header(stamp=now, frame_id="base_link"), []))
        if self.fake_phone and self.ticks % 50 == 0:
            self.heartbeat_pub.publish(Empty())  # a phone with the app open, walking along
        if self.indoor and self.ticks % 100 == 1:
            m = PoseWithCovarianceStamped()
            m.header.stamp, m.header.frame_id = now, "map"
            m.pose.pose.position.x, m.pose.pose.position.y = self.x, self.y
            m.pose.covariance[0] = m.pose.covariance[7] = 1.0
            self.fix_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
