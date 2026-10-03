"""Velocity commands to the motor microcontroller over USB serial (spec section 10)."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int32MultiArray

from .protocol import Button, Encoders, LineBuffer, Ultrasonic, format_velocity, parse_line
from .skid import compensate


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")
        self.port_name = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = self.declare_parameter("baud", 115200).value
        rate = self.declare_parameter("rate_hz", 20.0).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.3).value
        # Tank steering: the tracks slip sideways, so the car turns slower than
        # commanded. Measured with calibrate_turn (guide_bridge/skid.py).
        self.turn_gain = float(self.declare_parameter("turn_gain", 1.0).value)
        self.min_pivot = float(self.declare_parameter("min_pivot_rate", 0.0).value)
        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd = None
        self.serial = None
        self.buf = LineBuffer()
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.enc_pub = self.create_publisher(Int32MultiArray, "encoders", 10)
        self.ptt_pub = self.create_publisher(Bool, "voice/ptt", 10)
        self.range_pub = self.create_publisher(Range, "bumper/range", 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: Twist):
        self.linear, self.angular = msg.linear.x, msg.angular.z
        self.last_cmd = self.get_clock().now()

    def open(self) -> bool:
        if self.serial is not None:
            return True
        try:
            import serial

            self.serial = serial.Serial(self.port_name, self.baud, timeout=0)
            self.get_logger().info(f"opened {self.port_name}")
            return True
        except Exception as e:  # port missing or busy: retry next tick
            self.get_logger().warn(f"cannot open {self.port_name}: {e}", throttle_duration_sec=5.0)
            return False

    def tick(self):
        if not self.open():
            return
        lin, ang = self.linear, self.angular
        if self.last_cmd is None or (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            lin = ang = 0.0
        lin, ang = compensate(lin, ang, self.turn_gain, self.min_pivot)
        try:
            self.serial.write(format_velocity(lin, ang))
            data = self.serial.read(256)
        except Exception as e:
            self.get_logger().error(f"serial error, reopening: {e}")
            try:
                self.serial.close()
            finally:
                self.serial = None
            return
        for line in self.buf.feed(data):
            item = parse_line(line)
            if isinstance(item, Encoders):
                self.enc_pub.publish(Int32MultiArray(data=[item.left, item.right]))
            elif isinstance(item, Button):
                self.ptt_pub.publish(Bool(data=item.pressed))
            elif isinstance(item, Ultrasonic):
                r = Range()
                r.header.stamp, r.header.frame_id = self.get_clock().now().to_msg(), "bumper_link"
                r.radiation_type, r.field_of_view = Range.ULTRASOUND, 0.26  # HC-SR04: about 15 deg
                r.min_range, r.max_range, r.range = 0.02, 4.0, item.metres
                self.range_pub.publish(r)

    def stop(self):
        if self.serial is not None:
            try:
                self.serial.write(format_velocity(0.0, 0.0))
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
