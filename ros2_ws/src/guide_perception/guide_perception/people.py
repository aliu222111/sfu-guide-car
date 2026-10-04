"""Person detection (YOLO11n + ByteTrack) and distance to the nearest person ahead (spec section 7)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .ground import project_to_ground


@dataclass
class Person:
    track_id: int | None
    box: tuple[float, float, float, float]  # x1, y1, x2, y2 pixels
    conf: float
    x: float = math.inf  # metres ahead in base_link
    y: float = 0.0  # metres left

    @property
    def distance(self) -> float:
        return math.hypot(self.x, self.y)


def locate(people: list[Person], K, R, t, image_height: int, max_range: float = 15.0) -> list[Person]:
    """Place each person by projecting the bottom centre of their box (their feet) to the ground.

    If the box reaches the bottom edge the feet are out of view, so the person
    is nearer than any ground the camera sees: they are treated as right in
    front of the car (which stops it).
    """
    for p in people:
        x1, _, x2, y2 = p.box
        if y2 >= image_height - 2:
            p.x, p.y = 0.5, 0.0
            continue
        pts, ok = project_to_ground(K, R, t, np.array([(x1 + x2) / 2]), np.array([y2]), max_range)
        if ok[0]:
            p.x, p.y = float(pts[0, 0]), float(pts[0, 1])
    return people


def nearest_ahead(people: list[Person], half_width: float = 1.5) -> float:
    """Distance to the nearest person in the car's path; inf when none."""
    best = math.inf
    for p in people:
        if p.x > 0 and abs(p.y) <= half_width:
            best = min(best, p.distance)
    return best


class PersonDetector:
    """Ultralytics YOLO11 with ByteTrack. ``model`` may be a .pt (GPU) or an exported .onnx (CPU)."""

    def __init__(self, model: str = "yolo11n.pt", conf: float = 0.35, imgsz: int = 640, device=None):
        from ultralytics import YOLO

        self.model = YOLO(model)
        self.conf = conf
        self.imgsz = imgsz
        self.device = device

    def detect(self, bgr) -> list[Person]:
        res = self.model.track(
            bgr, persist=True, tracker="bytetrack.yaml", classes=[0], conf=self.conf, imgsz=self.imgsz, device=self.device, verbose=False
        )[0]
        out = []
        boxes = res.boxes
        if boxes is None:
            return out
        ids = boxes.id.int().tolist() if boxes.id is not None else [None] * len(boxes)
        for xyxy, c, tid in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), ids):
            out.append(Person(tid, tuple(xyxy), float(c)))
        return out
