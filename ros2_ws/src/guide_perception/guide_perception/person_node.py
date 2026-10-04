"""Person detection: ``people/nearest`` (Float32 metres, inf when none) at every frame,
plus ``people/detections`` (String, JSON) for the phone app and debugging."""

from __future__ import annotations

from pathlib import Path

import json
import math

from std_msgs.msg import Float32, String

from .base_node import CameraNode, spin
from .people import PersonDetector, locate, nearest_ahead


class PersonNode(CameraNode):
    def __init__(self):
        super().__init__("person_detection", default_rate=10.0)
        model = self.declare_parameter("model", "yolo11n.pt").value
        if not Path(model).exists():
            self.get_logger().fatal(f"model file {model} is missing: run scripts/export_models.py (README, Install)")
            raise SystemExit(1)
        conf = self.declare_parameter("conf", 0.35).value
        imgsz = self.declare_parameter("imgsz", 640).value
        device = self.declare_parameter("device", "").value or None
        self.half_width = self.declare_parameter("path_half_width", 1.5).value
        self.det = PersonDetector(model, conf, imgsz, device)
        self.nearest_pub = self.create_publisher(Float32, "people/nearest", 10)
        self.det_pub = self.create_publisher(String, "people/detections", 10)

    def process(self, bgr, header):
        people = self.det.detect(bgr)
        K, R, t, _ = self.geometry(bgr.shape)
        locate(people, K, R, t, bgr.shape[0])
        d = nearest_ahead(people, self.half_width)
        self.nearest_pub.publish(Float32(data=float(d)))
        self.det_pub.publish(
            String(data=json.dumps([{"id": p.track_id, "box": [round(v, 1) for v in p.box], "conf": round(p.conf, 2), "x": round(p.x, 2) if math.isfinite(p.x) else None, "y": round(p.y, 2)} for p in people]))
        )


def main():
    spin(PersonNode)
