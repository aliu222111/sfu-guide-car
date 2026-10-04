"""Person detection (YOLO11n + ByteTrack) and distance to the nearest person ahead (spec section 7)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .ground import project_to_ground


@dataclass
class Person:
    """A YOLO detection (a person, or with ``cls`` another COCO class) placed on the floor."""

    track_id: int | None
    box: tuple[float, float, float, float]  # x1, y1, x2, y2 pixels
    conf: float
    x: float = math.inf  # metres ahead in base_link
    y: float = 0.0  # metres left
    cls: int = 0  # COCO class: 0 is a person
    label: str = "person"

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

    def class_ids(self, labels) -> list[int]:
        """The model's class numbers for these names (whichever of them it knows)."""
        want = set(labels)
        return [k for k, n in self.model.names.items() if n in want]

    def detect(self, bgr, classes=None) -> list[Person]:
        """Tracked detections in one pass: ``classes`` are class numbers (class_ids); default people."""
        if classes is None:
            classes = self.class_ids(["person"])
        res = self.model.track(
            bgr, persist=True, tracker="bytetrack.yaml", classes=list(classes), conf=self.conf, imgsz=self.imgsz,
            device=self.device, verbose=False,
        )[0]
        out = []
        boxes = res.boxes
        if boxes is None:
            return out
        names = res.names
        ids = boxes.id.int().tolist() if boxes.id is not None else [None] * len(boxes)
        for xyxy, c, k, tid in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.int().tolist(), ids):
            label = names.get(k, str(k))
            out.append(Person(tid, tuple(xyxy), float(c), cls=0 if label == "person" else k + 1000, label=label))
        return out


# Things that stand on the floor and can block a small car (spec section 7): they go into the
# costmaps where they touch the floor. People are handled by the velocity gate instead. Names
# cover both the standard COCO model (yolo11n) and the school model (SCHOOL_CLASSES).
FLOOR_OBJECTS = {
    "bicycle", "motorcycle", "bench", "cat", "dog", "backpack", "umbrella", "handbag", "suitcase", "skateboard",
    "chair", "couch", "potted plant", "dining table", "fire hydrant",
    "desk", "table", "stool", "trash can", "recycling bin", "wet floor sign", "traffic cone", "cart",
    "cardboard box", "bag", "scooter", "stroller", "wheelchair", "podium", "whiteboard", "easel",
    "sign board", "ladder", "vending machine", "water fountain", "fire extinguisher", "stairs",
}

# The school model: an open-vocabulary YOLOE exported with these names built in
# (scripts/export_models.py --only school). People first, then what a campus corridor holds.
SCHOOL_CLASSES = [
    "person", "chair", "desk", "table", "stool", "bench", "couch", "trash can", "recycling bin",
    "wet floor sign", "traffic cone", "cart", "cardboard box", "backpack", "bag", "suitcase", "bicycle",
    "scooter", "skateboard", "stroller", "wheelchair", "potted plant", "podium", "whiteboard", "easel",
    "sign board", "ladder", "vending machine", "water fountain", "fire extinguisher", "stairs", "door",
    "elevator door", "dog",
]


def carried(obj: Person, people: list[Person], inside: float = 0.6) -> bool:
    """True when most of an object's box lies inside a person's: a worn backpack, a held bag. Its
    bottom edge is in mid-air, so placing it on the floor would put a ghost obstacle behind them."""
    x1, y1, x2, y2 = obj.box
    area = max(1e-6, (x2 - x1) * (y2 - y1))
    for p in people:
        px1, py1, px2, py2 = p.box
        w, h = min(x2, px2) - max(x1, px1), min(y2, py2) - max(y1, py1)
        if w > 0 and h > 0 and w * h / area >= inside:
            return True
    return False


def object_footprint(obj: Person, K, cam_xy=(0.0, 0.0), step: float = 0.05) -> list[tuple[float, float]]:
    """Points across the bottom edge of a placed object (base_link x, y): its width at its distance
    from the camera at ``cam_xy`` (base_link)."""
    if not math.isfinite(obj.x) or obj.x <= 0:
        return []
    x1, _, x2, _ = obj.box
    half = (x2 - x1) / 2 * math.hypot(obj.x - cam_xy[0], obj.y - cam_xy[1]) / K[0, 0]
    n = max(2, int(2 * half / step) + 1)
    # across the line of sight (approximately sideways in base_link for objects ahead)
    bearing = math.atan2(obj.y, obj.x)
    return [(obj.x - math.sin(bearing) * o, obj.y + math.cos(bearing) * o) for o in np.linspace(-half, half, n)]
