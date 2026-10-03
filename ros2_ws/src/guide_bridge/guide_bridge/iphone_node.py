"""The iPhone as the front camera over USB, through the Record3D app (spec section 19).

Publishes, stamped on arrival:
  front/image_rect    bgr8, ``out_width`` wide (ARKit images are already undistorted)
  front/camera_info
  front/depth         32FC1 metres, registered to front/image_rect, 0 = unknown
  front/points        LiDAR points in the camera's optical frame, for the costmaps
  /vo                 ARKit visual-inertial odometry as base_link velocity (and
                      pose, which the EKFs ignore; with publish_tf it is also the
                      odom -> base_link transform for the handheld mapping rig)

On the phone: Record3D, Settings, "USB Streaming mode" on, then press record.
On the laptop: ``pip install record3d`` (and libusbmuxd).
"""

from __future__ import annotations

import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from .arkit import TwistTracker, base_pose, depth_points, prepare

DEVICE_TYPE_LIDAR = 1


class IphoneNode(Node):
    def __init__(self):
        super().__init__("iphone")
        P = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.device_index = int(P("device_index", 0))
        self.out_width = int(P("out_width", 640))
        self.min_conf = int(P("min_confidence", 2))
        self.max_depth = float(P("max_depth", 5.0))
        self.stride = int(P("point_stride", 2))
        self.points_period = 1.0 / float(P("points_rate", 10.0))
        self.mount = (float(P("mount_x", 0.20)), float(P("mount_y", 0.0)))
        self.turns = int(P("quarter_turns", 0))
        self.frame = P("frame_id", "iphone_camera_optical_frame")
        self.publish_tf = bool(P("publish_tf", False))
        self.vel_var = float(P("velocity_variance", 0.0025))
        self.tracker = TwistTracker(window=float(P("twist_window", 0.1)))
        self.img_pub = self.create_publisher(Image, "front/image_rect", 5)
        self.info_pub = self.create_publisher(CameraInfo, "front/camera_info", 5)
        self.depth_pub = self.create_publisher(Image, "front/depth", 5)
        self.points_pub = self.create_publisher(PointCloud2, "front/points", 5)
        self.vo_pub = self.create_publisher(Odometry, "/vo", 20)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.last_points = -math.inf
        self.frame_ready = threading.Event()
        self.stopped = threading.Event()
        self.session = None
        threading.Thread(target=self._run, daemon=True).start()

    # --- Record3D --------------------------------------------------------------------
    def _connect(self) -> bool:
        from record3d import Record3DStream

        devs = Record3DStream.get_connected_devices()
        if len(devs) <= self.device_index:
            return False
        self.session = Record3DStream()
        self.session.on_new_frame = self.frame_ready.set
        self.session.on_stream_stopped = self.stopped.set
        self.stopped.clear()
        self.session.connect(devs[self.device_index])
        self.get_logger().info("iPhone connected")
        return True

    def _run(self):
        warned = False
        while rclpy.ok():
            try:
                connected = self._connect()
            except Exception as e:  # library missing, USB trouble
                connected = False
                if not warned:
                    self.get_logger().error(f"Record3D: {e}")
            if not connected:
                if not warned:
                    self.get_logger().warn("no iPhone streaming: open Record3D, USB streaming mode, press record")
                    warned = True
                self.stopped.wait(2.0)
                continue
            warned = False
            while rclpy.ok() and not self.stopped.is_set():
                if not self.frame_ready.wait(0.5):
                    continue
                self.frame_ready.clear()
                s = self.session
                try:
                    k = s.get_intrinsic_mat()
                    pose = s.get_camera_pose()
                    self.handle_frame(s.get_rgb_frame(), s.get_depth_frame(), s.get_confidence_frame(),
                                      np.array([[k.fx, 0, k.tx], [0, k.fy, k.ty], [0, 0, 1.0]]),
                                      (pose.qx, pose.qy, pose.qz, pose.qw, pose.tx, pose.ty, pose.tz))
                except Exception as e:
                    self.get_logger().error(f"frame: {e}", throttle_duration_sec=5.0)
            self.get_logger().warn("iPhone stream stopped; reconnecting")
            self.stopped.clear()
            self.tracker.hist.clear()

    # --- Conversion ------------------------------------------------------------------
    def handle_frame(self, rgb, depth, conf, K, pose):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        stamp = now.to_msg()
        f = prepare(rgb, depth, conf, K, self.out_width, self.min_conf, self.max_depth, self.turns)
        header = Header(stamp=stamp, frame_id=self.frame)
        h, w = f.bgr.shape[:2]
        img = Image(header=header, height=h, width=w, encoding="bgr8", step=w * 3, data=f.bgr.tobytes())
        dep = Image(header=header, height=h, width=w, encoding="32FC1", step=w * 4, data=f.depth.tobytes())
        info = CameraInfo(header=header, width=w, height=h, distortion_model="plumb_bob")
        info.d = [0.0] * 5
        info.k = [float(v) for v in f.K.flatten()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f.K[0, 0], 0.0, f.K[0, 2], 0.0, 0.0, f.K[1, 1], f.K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.img_pub.publish(img)
        self.info_pub.publish(info)
        self.depth_pub.publish(dep)
        if t - self.last_points >= self.points_period:
            self.last_points = t
            self.points_pub.publish(point_cloud2.create_cloud_xyz32(header, depth_points(f.depth_small, f.K_small, self.stride)))
        self.publish_odometry(t, stamp, *pose)

    def publish_odometry(self, t, stamp, qx, qy, qz, qw, tx, ty, tz):
        x, y, yaw = base_pose(qx, qy, qz, qw, tx, ty, tz, *self.mount)
        tw = self.tracker.update(t, x, y, yaw)
        if tw is None:
            return
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, "odom", "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance = [0.0] * 36
        for i in range(6):
            m.pose.covariance[i * 7] = 0.01
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z = tw.vx, tw.vy, tw.wz
        m.twist.covariance = [0.0] * 36
        for i in range(6):
            m.twist.covariance[i * 7] = self.vel_var
        self.vo_pub.publish(m)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
            tf.transform.translation.x, tf.transform.translation.y = x, y
            tf.transform.rotation = m.pose.pose.orientation
            self.tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = IphoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
