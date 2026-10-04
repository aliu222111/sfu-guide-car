"""Velocity commands to the RC car, through the microcontroller that works its hand
controller, over USB serial (spec section 10).

/cmd_vel (after the velocity gate) becomes steering and throttle (ackermann.py,
calibrated in guide.yaml), with the open-loop throttle trimmed on the measured
speed (speed.py). The microcontroller centres both if the lines stop.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from .ackermann import Ackermann, CarConfig
from .protocol import Button, LineBuffer, Ultrasonic, format_drive, parse_line
from .speed import SpeedTrim


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")
        self.port_name = self.declare_parameter("port", "/dev/ttyUSB0").value  # an ESP32 dev board (CP2102 / CH340)
        self.baud = self.declare_parameter("baud", 115200).value
        rate = self.declare_parameter("rate_hz", 20.0).value
        self.cmd_timeout = self.declare_parameter("cmd_timeout", 0.3).value
        cfg = CarConfig()
        for name, default in vars(CarConfig()).items():
            setattr(cfg, name, type(default)(self.declare_parameter(name, default).value))
        self.car = Ackermann(cfg)
        # The RC throttle is open loop: trim it on the measured speed (speed.py).
        self.trim = SpeedTrim(gain=float(self.declare_parameter("speed_trim_gain", 0.8).value),
                              limit=float(self.declare_parameter("speed_trim_limit", 0.3).value), stop=cfg.stop_speed)
        self.period = 1.0 / rate
        self.measured = None  # (forward speed m/s, time)
        self.linear = 0.0
        self.angular = 0.0
        self.last_cmd = None
        self.serial = None
        self.buf = LineBuffer()
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(Odometry, "odometry/local", self.on_odom, 10)
        self.ptt_pub = self.create_publisher(Bool, "voice/ptt", 10)
        self.range_pub = self.create_publisher(Range, "bumper/range", 10)
        self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: Twist):
        self.linear, self.angular = msg.linear.x, msg.angular.z
        self.last_cmd = self.get_clock().now()

    def on_odom(self, msg: Odometry):
        self.measured = (msg.twist.twist.linear.x, self.now())

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def open(self) -> bool:
        if self.serial is not None:
            return True
        try:
            import serial

            # dtr/rts low before opening: asserting them resets an ESP32 dev board, which
            # would leave the controller's wiper inputs floating for a second on every open.
            s = serial.Serial()
            s.port, s.baudrate, s.timeout = self.port_name, self.baud, 0
            s.dtr = s.rts = False
            s.open()
            self.serial = s
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
        speed, age = (self.measured[0], self.now() - self.measured[1]) if self.measured else (None, 0.0)
        lin, ang = self.trim.command(lin, ang, speed, age, self.period)
        try:
            self.serial.write(format_drive(*self.car.command(lin, ang)))
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
            if isinstance(item, Button):
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
                self.serial.write(format_drive(0.0, 0.0))
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
