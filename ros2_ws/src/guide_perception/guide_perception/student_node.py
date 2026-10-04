"""Rear camera student tracking (spec section 9, phase 6).

Locks on to the person following the car (the nearest person behind it when a
trip starts, by ByteTrack id) and publishes ``guide/student_distance``
(Float32 metres, inf when not seen). The mission's follow monitor treats
"not seen for a while" or "too far" as not following.
"""

from __future__ import annotations

from pathlib import Path

import math

from std_msgs.msg import Float32, String

from .base_node import CameraNode, spin
from .people import PersonDetector, locate


def pick_student(people, current_id, lost_for: float = 0.0, last_distance: float | None = None,
                 max_lock_distance: float = 3.0, relock_after: float = 3.0, relock_window: float = 1.0):
    """The student among ``people``, or None when not in view.

    No lock yet (trip start): the nearest person within range. Locked: only
    that track id. A short occlusion must not hand the lock to a passer-by, so
    after ``relock_after`` seconds out of view (the tracker may have given the
    student a new id) re-lock only onto someone near the student's last distance.
    """
    for p in people:
        if current_id is not None and p.track_id == current_id:
            return p
    near = [p for p in people if p.distance <= max_lock_distance and p.track_id is not None]
    if current_id is None:
        return min(near, key=lambda p: p.distance) if near else None
    if lost_for < relock_after or last_distance is None:
        return None
    near = [p for p in near if abs(p.distance - last_distance) <= relock_window]
    return min(near, key=lambda p: abs(p.distance - last_distance)) if near else None


class StudentNode(CameraNode):
    def __init__(self):
        super().__init__("student_tracker", default_rate=5.0)
        model = self.declare_parameter("model", "yolo11n.pt").value
        if not Path(model).exists():
            self.get_logger().fatal(f"model file {model} is missing: run scripts/export_models.py (README, Install)")
            raise SystemExit(1)
        self.det = PersonDetector(model)
        self.lock_distance = self.declare_parameter("lock_distance", 3.0).value
        self.student_id = None
        self.seen_at = 0.0
        self.last_distance = None
        self.pub = self.create_publisher(Float32, "guide/student_distance", 10)
        self.create_subscription(String, "guide/destination", self.on_new_trip, 10)

    def on_new_trip(self, _msg):
        self.student_id, self.last_distance = None, None  # lock on afresh

    def process(self, bgr, header):
        pose = self.camera_pose()
        if self.K is None or pose is None:
            return
        people = locate(self.det.detect(bgr), self.K, *pose, bgr.shape[0])
        now = self.get_clock().now().nanoseconds * 1e-9
        s = pick_student(people, self.student_id, now - self.seen_at, self.last_distance, self.lock_distance)
        if s:
            self.student_id, self.seen_at, self.last_distance = s.track_id, now, s.distance
        # The rear camera's frame points backwards, so distance is what matters, not sign.
        self.pub.publish(Float32(data=float(s.distance) if s and math.isfinite(s.distance) else math.inf))


def main():
    spin(StudentNode)
