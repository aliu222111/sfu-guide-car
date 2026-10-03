"""Is the student still with the car? (spec section 9)

Phone heartbeat every second: two missed (more than ``heartbeat_timeout``
seconds without one) means not following. Bluetooth signal strength, when
available, is a second check: below ``rssi_threshold`` for ``rssi_hold``
seconds means not following. Without a phone (voice mode) the car relies on the
walking-pace limit and the "still with me?" prompts, plus the rear camera
when fitted (phase 6), which tracks the student directly in either mode.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class FollowParams:
    heartbeat_timeout: float = 2.5  # 1 Hz heartbeats: two missed (spec section 9)
    rssi_threshold: float = -80.0  # dBm, calibrate per phone
    rssi_hold: float = 3.0
    student_max_distance: float = 6.0  # m behind the car, rear camera (phase 6)
    student_lost_after: float = 4.0  # s without seeing the student


class FollowMonitor:
    def __init__(self, params: FollowParams | None = None):
        self.p = params or FollowParams()
        self.last_heartbeat = -math.inf
        self.rssi_low_since: float | None = None
        self.rssi_seen = False
        self.voice_mode = False
        self.camera_seen = False
        self.last_student = -math.inf
        self.student_far_since: float | None = None

    def heartbeat(self, now: float):
        self.last_heartbeat = now

    def rssi(self, dbm: float, now: float):
        self.rssi_seen = True
        if dbm < self.p.rssi_threshold:
            if self.rssi_low_since is None:
                self.rssi_low_since = now
        else:
            self.rssi_low_since = None

    def student(self, distance: float, now: float):
        """Rear-camera distance to the tracked student; inf when not in view."""
        self.camera_seen = True
        if math.isfinite(distance):
            self.last_student = now
            if distance > self.p.student_max_distance:
                self.student_far_since = self.student_far_since or now
            else:
                self.student_far_since = None

    def following(self, now: float) -> bool:
        if self.camera_seen:
            if now - self.last_student > self.p.student_lost_after:
                return False
            if self.student_far_since is not None and now - self.student_far_since >= self.p.rssi_hold:
                return False
        if self.voice_mode:
            return True
        if now - self.last_heartbeat > self.p.heartbeat_timeout:
            return False
        if self.rssi_low_since is not None and now - self.rssi_low_since >= self.p.rssi_hold:
            return False
        return True
