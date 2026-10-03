"""Rear camera student tracking (spec section 9, phase 6).

Locks on to the person following the car (the nearest person behind it when a
trip starts, by ByteTrack id) and publishes ``guide/student_distance``
(Float32 metres, inf when not seen). The mission's follow monitor treats
"not seen for a while" or "too far" as not following.
"""

from __future__ import annotations

import math

from std_msgs.msg import Float32, String

from .base_node import CameraNode, spin
from .people import PersonDetector, locate


def pick_student(people, current_id, max_lock_distance: float = 3.0):
    """Keep the tracked id if still visible; otherwise lock on to the nearest person within range."""
    for p in people:
        if current_id is not None and p.track_id == current_id:
            return p
    near = [p for p in people if p.distance <= max_lock_distance and p.track_id is not None]
    return min(near, key=lambda p: p.distance) if near else None


class StudentNode(CameraNode):
    def __init__(self):
        super().__init__("student_tracker", default_rate=5.0)
        self.det = PersonDetector(self.declare_parameter("model", "yolo11n.pt").value)
        self.lock_distance = self.declare_parameter("lock_distance", 3.0).value
        self.student_id = None
        self.pub = self.create_publisher(Float32, "guide/student_distance", 10)
        self.create_subscription(String, "guide/destination", lambda _: setattr(self, "student_id", None), 10)

    def process(self, bgr, header):
        pose = self.camera_pose()
        if self.K is None or pose is None:
            return
        people = locate(self.det.detect(bgr), self.K, *pose, bgr.shape[0])
        s = pick_student(people, self.student_id, self.lock_distance)
        self.student_id = s.track_id if s else self.student_id
        # The rear camera's frame points backwards, so distance is what matters, not sign.
        self.pub.publish(Float32(data=float(s.distance) if s and math.isfinite(s.distance) else math.inf))


def main():
    spin(StudentNode)
