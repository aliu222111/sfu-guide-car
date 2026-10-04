"""Floor segmentation to costmap points (spec section 7).

Publishes ``floor/obstacles`` (where drivable floor ends in each image column,
projected to the ground) and ``floor/free`` (drivable ground in front of it,
for clearing). Both are PointCloud2 in base_link for Nav2's obstacle layer.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from .base_node import CameraNode, spin
from .ground import free_boundary, project_to_ground
from .segmentation import FloorSegmenter


class FloorSegNode(CameraNode):
    def __init__(self):
        super().__init__("floor_segmentation", default_rate=2.0)
        model = self.declare_parameter("model", "models/segformer-b0-ade.onnx").value
        if not Path(model).exists():
            self.get_logger().fatal(f"model file {model} is missing: run scripts/export_models.py (README, Install)")
            raise SystemExit(1)
        labels = self.declare_parameter("labels", "models/segformer-b0-ade.labels.json").value
        w = self.declare_parameter("input_width", 320).value
        h = self.declare_parameter("input_height", 180).value
        self.crop_top = self.declare_parameter("crop_top", 0.45).value  # fraction of the image
        self.max_range = self.declare_parameter("max_range", 5.0).value
        self.obstacle_height = self.declare_parameter("obstacle_height", 0.1).value
        self.step = self.declare_parameter("column_step", 4).value
        self.seg = FloorSegmenter(model, (w, h), labels_path=labels)
        self.obs_pub = self.create_publisher(PointCloud2, "floor/obstacles", 5)
        self.free_pub = self.create_publisher(PointCloud2, "floor/free", 5)

    def process(self, bgr, header):
        pose = self.camera_pose()
        if self.K is None or pose is None:
            return
        R, t = pose
        top = int(bgr.shape[0] * self.crop_top)
        mask = np.zeros(bgr.shape[:2], dtype=bool)
        mask[top:] = self.seg.drivable_mask(bgr[top:])
        cols, rows, hit = free_boundary(mask, start_row=top, step=self.step)
        pts, ok = project_to_ground(self.K, R, t, cols[hit], rows[hit], self.max_range)
        obstacles = pts[ok].copy()
        obstacles[:, 2] = self.obstacle_height
        # Free samples: a few rows between the bottom and each boundary.
        fu, fv = [], []
        for c, r in zip(cols, rows):
            for v in np.linspace(bgr.shape[0] - 1, max(r + 2, top), 6):
                fu.append(c)
                fv.append(v)
        fpts, fok = project_to_ground(self.K, R, t, np.array(fu), np.array(fv), self.max_range)
        free = fpts[fok]
        header.frame_id = self.base_frame
        self.obs_pub.publish(point_cloud2.create_cloud_xyz32(header, obstacles.tolist()))
        self.free_pub.publish(point_cloud2.create_cloud_xyz32(header, free.tolist()))


def main():
    spin(FloorSegNode)
