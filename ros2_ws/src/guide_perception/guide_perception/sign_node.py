"""Room-sign reader: publishes ``signs`` (String, JSON ``{"text": "AQ 3150", "conf": 0.93, "bearing": 0.2}``)
per read; ``bearing`` (radians from the camera axis, left positive) only when the backend gives text boxes."""

from __future__ import annotations

import json
import math

from std_msgs.msg import Bool, String

from .base_node import CameraNode, spin
from .sign_reader import ClaudeBackend, ClaudeSubscriptionBackend, PaddleBackend


class SignNode(CameraNode):
    def __init__(self):
        super().__init__("sign_reader", default_rate=2.0)
        backend = self.declare_parameter("backend", "paddle").value
        model = self.declare_parameter("claude_model", "claude-haiku-4-5").value
        if backend == "claude":  # the owner's Claude subscription
            self.reader = ClaudeSubscriptionBackend(model)
        elif backend == "claude_api":  # an Anthropic API key
            self.reader = ClaudeBackend(model)
        elif backend == "paddle":
            self.reader = PaddleBackend(self.declare_parameter("min_conf", 0.8).value)
        else:
            raise ValueError(f"backend must be paddle, claude or claude_api, not {backend!r}")
        self.indoor = False
        self.create_subscription(Bool, "guide/outdoor", lambda m: setattr(self, "indoor", not m.data), 10)
        self.pub = self.create_publisher(String, "signs", 10)

    def process(self, bgr, header):
        if not self.indoor:  # spec: used only indoors
            return
        try:
            reads = self.reader.read(bgr)
        except Exception as e:  # network loss for the cloud backend: no reads (spec section 16)
            self.get_logger().warn(f"sign read failed: {e}", throttle_duration_sec=10.0)
            return
        for r in reads:
            doc = {"text": r.text, "conf": r.conf, "stamp": header.stamp.sec + header.stamp.nanosec * 1e-9}
            if r.u is not None and self.K is not None:
                # Bearing of the sign from the camera axis, radians, left positive.
                doc["bearing"] = math.atan2(-(r.u - self.K[0, 2]), self.K[0, 0])
            self.pub.publish(String(data=json.dumps(doc)))


def main():
    spin(SignNode)
