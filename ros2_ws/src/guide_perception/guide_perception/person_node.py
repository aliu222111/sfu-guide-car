"""Person and object detection, one YOLO pass per frame.

``people/nearest`` (Float32 metres, inf when none) for the velocity gate, ``people/detections``
(String, JSON) for debugging; with ``objects`` on, floor-standing objects (chairs, benches,
bags, bikes, plants, ...: people.FLOOR_OBJECTS) go out as ``objects/obstacles`` (PointCloud2 in
base_link, where they touch the floor, for Nav2's obstacle layer) and every detection as
``objects/detections`` (String, JSON: label, conf, x, y)."""

from __future__ import annotations

from pathlib import Path

import json
import math

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, String

from .base_node import CameraNode, spin
from .people import FLOOR_OBJECTS, PersonDetector, carried, locate, nearest_ahead, object_footprint


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
        self.objects = self.declare_parameter("objects", True).value
        self.object_range = self.declare_parameter("object_max_range", 5.0).value
        self.classes = self.det.class_ids({"person"} | (FLOOR_OBJECTS if self.objects else set()))
        self.get_logger().info(f"detecting {sorted(self.det.model.names[k] for k in self.classes)}")
        self.obj_pub = self.create_publisher(PointCloud2, "objects/obstacles", 5)
        self.obj_det_pub = self.create_publisher(String, "objects/detections", 10)

    def process(self, bgr, header):
        found = self.det.detect(bgr, self.classes)
        K, R, t, _ = self.geometry(bgr.shape)
        locate(found, K, R, t, bgr.shape[0])
        people = [p for p in found if p.label == "person"]
        objects = [o for o in found if o.label in FLOOR_OBJECTS]
        d = nearest_ahead(people, self.half_width)
        self.nearest_pub.publish(Float32(data=float(d)))
        if self.objects:
            pts = []
            for o in objects:
                # Cut off by the bottom edge: locate() puts it at 0.5 m; leave those to stereo and the bumper
                if o.box[3] < bgr.shape[0] - 2 and o.x <= self.object_range and not carried(o, people):
                    pts += [(x, y, 0.1) for x, y in object_footprint(o, K, t[:2])]
            hdr = type(header)()
            hdr.stamp, hdr.frame_id = header.stamp, self.base_frame
            self.obj_pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))
            self.obj_det_pub.publish(String(data=json.dumps(
                [{"label": o.label, "conf": round(o.conf, 2), "x": round(o.x, 2) if math.isfinite(o.x) else None,
                  "y": round(o.y, 2)} for o in found])))
        self.det_pub.publish(
            String(data=json.dumps([{"id": p.track_id, "box": [round(v, 1) for v in p.box], "conf": round(p.conf, 2), "x": round(p.x, 2) if math.isfinite(p.x) else None, "y": round(p.y, 2)} for p in people]))
        )


def main():
    spin(PersonNode)
