"""Shared plumbing for camera nodes: keep only the newest frame, process at a fixed rate."""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .ground import quat_to_matrix


class CameraNode(Node):
    """Subclasses implement ``process(bgr, header)``; it runs on a worker thread."""

    def __init__(self, name: str, default_rate: float):
        super().__init__(name)
        self.rate = self.declare_parameter("rate_hz", default_rate).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.bridge = CvBridge()
        self.K = None
        self.camera_frame = None
        self._R_t = None
        self._latest = None
        self._lock = threading.Lock()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Image, "image", self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "camera_info", self._on_info, qos_profile_sensor_data)
        self._stop = False
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    def _on_info(self, msg: CameraInfo):
        # Rectified images: the projection matrix P holds the intrinsics.
        P = np.array(msg.p, dtype=float).reshape(3, 4)
        self.K = P[:, :3] if P[0, 0] > 0 else np.array(msg.k, dtype=float).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def _on_image(self, msg: Image):
        with self._lock:
            self._latest = msg

    def camera_pose(self):
        """(R, t) of the camera optical frame in base_link; cached once found (the mount is rigid)."""
        if self._R_t is None and self.camera_frame:
            try:
                tr = self.tf.lookup_transform(self.base_frame, self.camera_frame, Time()).transform
                R = quat_to_matrix(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
                self._R_t = (R, np.array([tr.translation.x, tr.translation.y, tr.translation.z]))
            except TransformException as e:
                self.get_logger().warn(f"no transform {self.base_frame} <- {self.camera_frame}: {e}", throttle_duration_sec=5.0)
        return self._R_t

    def _loop(self):
        period = 1.0 / max(self.rate, 0.1)
        while not self._stop and rclpy.ok():
            t0 = time.monotonic()
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is not None:
                try:
                    bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.process(bgr, msg.header)
                except Exception as e:  # keep the node alive; log and carry on
                    self.get_logger().error(f"{type(e).__name__}: {e}", throttle_duration_sec=2.0)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def process(self, bgr, header):
        raise NotImplementedError

    def destroy_node(self):
        self._stop = True
        return super().destroy_node()


def spin(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
