"""Velocity gate node: Nav2's cmd_vel_nav plus mission inputs in, cmd_vel out."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32, String

from .velocity_gate import GateParams, VelocityGate


class VelocityGateNode(Node):
    def __init__(self):
        super().__init__("velocity_gate")
        p = GateParams()
        for name in vars(p):
            setattr(p, name, self.declare_parameter(name, getattr(p, name)).value)
        self.gate = VelocityGate(p)
        self.create_subscription(Twist, "cmd_vel_nav", lambda m: self.gate.nav_cmd(m.linear.x, m.angular.z, self.now()), 10)
        self.create_subscription(Twist, "guide/cmd_vel_manual", lambda m: self.gate.manual_cmd(m.linear.x, m.angular.z, self.now()), 10)
        self.create_subscription(Bool, "guide/drive_enable", lambda m: self.gate.set_enabled(m.data, self.now()), 10)
        self.create_subscription(Bool, "guide/recovery_rotate", lambda m: self.gate.set_recovery(m.data, self.now()), 10)
        self.create_subscription(Bool, "guide/outdoor", lambda m: self.gate.set_outdoor(m.data, self.now()), 10)
        self.create_subscription(Bool, "phone/alive", lambda m: self.gate.set_phone_alive(m.data, self.now()), 10)
        self.create_subscription(Float32, "people/nearest", lambda m: self.gate.person(m.data, self.now()), 10)
        self.create_subscription(Range, "bumper/range", lambda m: self.gate.bumper(m.range, self.now()), 10)
        # Collision detection: the phone's accelerometer (base_link axes) and the measured speed
        self.create_subscription(Imu, "phone/imu", lambda m: self.gate.accel(m.linear_acceleration.x, m.linear_acceleration.y, self.now()), 50)
        self.create_subscription(Odometry, "odometry/local", lambda m: self.gate.odom_speed(m.twist.twist.linear.x, self.now()), 10)
        self.collision_pub = self.create_publisher(String, "guide/collision", 10)
        self.collisions = 0
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.reason_pub = self.create_publisher(String, "guide/gate_reason", 10)
        self.last_reason = None
        self.create_timer(1.0 / self.declare_parameter("rate_hz", 20.0).value, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        lin, ang, why = self.gate.step(self.now())
        t = Twist()
        t.linear.x, t.angular.z = float(lin), float(ang)
        self.pub.publish(t)
        if self.gate.s.collisions != self.collisions:
            self.collisions = self.gate.s.collisions
            self.get_logger().warn(f"collision ({self.gate.s.collision_kind}): forward motion blocked for {self.gate.p.collision_hold:.0f} s")
            self.collision_pub.publish(String(data=self.gate.s.collision_kind))
        if why != self.last_reason:
            self.reason_pub.publish(String(data=why))
            self.last_reason = why


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
