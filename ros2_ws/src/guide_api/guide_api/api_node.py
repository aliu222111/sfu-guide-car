"""The rider app's API as a ROS node on the car's laptop.

    phone  --HTTP + server-sent events-->  this node  --/guide/* topics-->  mission
    mission -> Nav2 -> velocity gate -> serial bridge -> ESP32 -> the RC controller

It publishes the five topics the mission listens to and subscribes to its three
outputs (ros2_ws/README.md, "Topics"); everything else is in ``service.py``.
"""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String

from .chat import Concierge
from .frames import AppFrames
from .http_api import serve
from .service import CarLink, ScoutService

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)  # as the mission publishes /guide/route
CONFIG = Path(__file__).resolve().parent / "config"


class ApiNode(Node, CarLink):
    def __init__(self):
        super().__init__("scout_api")
        maps_dir = Path(self.declare_parameter("maps_dir", "maps").value)
        port = self.declare_parameter("port", 8090).value
        token = self.declare_parameter("token", "").value
        frames = self.declare_parameter("app_frames", "").value
        speed = self.declare_parameter("speed", 0.5).value            # the car's pace, for the time left
        ride_s = self.declare_parameter("elevator_s", 60.0).value

        self.dest_pub = self.create_publisher(String, "guide/destination", 10)
        self.confirm_pub = self.create_publisher(Empty, "guide/confirm", 10)
        self.cancel_pub = self.create_publisher(Empty, "guide/cancel", 10)
        self.pause_pub = self.create_publisher(Bool, "guide/pause", 10)
        self.heartbeat_pub = self.create_publisher(Empty, "guide/heartbeat", 10)

        dest_file = maps_dir / "destinations.json"
        destinations = json.loads(dest_file.read_text()) if dest_file.is_file() else {}
        if not destinations:
            self.get_logger().warning(f"no destinations in {dest_file}: rooms will not be checked")
        self.service = ScoutService(
            self, destinations, AppFrames.load(frames or CONFIG / "app_frames.json"),
            speed=speed, ride_s=ride_s,
            car_id=self.declare_parameter("car_id", "scout-01").value,
            car_name=self.declare_parameter("car_name", "Scout").value,
        )

        self.create_subscription(String, "guide/state", lambda m: self._json(m, self.service.on_state), 10)
        self.create_subscription(String, "guide/route", lambda m: self._json(m, self.service.on_route), LATCHED)
        self.create_subscription(String, "guide/speech", lambda m: self.service.on_speech(m.data), 10)
        self.create_timer(1.0, self.service.tick)

        chat = Concierge.from_env(destinations)   # needs GEMINI_API_KEY and ELEVENLABS_API_KEY, as the voice node does
        if chat is None:
            self.get_logger().info("Chat with ElevenLabs is off: no GEMINI_API_KEY")
        self.server = serve(self.service, port, token=token, chat=chat)
        self.get_logger().info(f"rider API on http://<this laptop>:{port}/")

    def _json(self, msg: String, sink):
        try:
            sink(json.loads(msg.data))
        except ValueError:
            self.get_logger().warning("dropped a message that was not JSON")

    # --- CarLink --------------------------------------------------------------
    def send_destination(self, key: str):
        self.dest_pub.publish(String(data=key))

    def send_confirm(self):
        self.confirm_pub.publish(Empty())

    def send_cancel(self):
        self.cancel_pub.publish(Empty())

    def send_pause(self, paused: bool):
        self.pause_pub.publish(Bool(data=bool(paused)))

    def send_heartbeat(self):
        self.heartbeat_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = ApiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
