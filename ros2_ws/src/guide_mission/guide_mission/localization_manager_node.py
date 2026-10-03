"""Switches localization between outdoors and a floor (spec section 6, "Transitions").

The global EKF (robot_localization, world frame ``map``) always runs. This node
decides what it may fuse:

- outdoors: GPS (``odometry/gps`` from navsat_transform -> ``guide/gps_odom``)
  and the phone's compass yaw (``phone/imu`` -> ``guide/imu_abs``);
- indoors: RTAB-Map localization poses converted to the GIS frame with the
  floor's alignment (``rtabmap/localization_pose`` -> ``guide/indoor_pose``) and
  sign fixes; the compass is dropped because steel and concrete distort it.

It loads the floor's occupancy grid into Nav2's map_server, asks RTAB-Map to
switch database when ``rtabmap_msgs`` is available, forwards pose resets, and
publishes ``guide/localized``: false when visual odometry is lost (spec section
6, "Without encoders") or, indoors, until a loop closure or sign fix has
placed the car on the current floor since the last switch or odometry loss.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from .alignment import Similarity2D, load_alignments

LOST_COVARIANCE = 9999.0  # rtabmap odometry reports this when tracking is lost


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _set_yaw(q, yaw):
    q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def transform_pose(msg: PoseWithCovarianceStamped, sim: Similarity2D, frame: str) -> PoseWithCovarianceStamped:
    out = PoseWithCovarianceStamped()
    out.header.stamp = msg.header.stamp
    out.header.frame_id = frame
    p = msg.pose.pose
    x, y = sim.apply(p.position.x, p.position.y)
    out.pose.pose.position.x, out.pose.pose.position.y = x, y
    _set_yaw(out.pose.pose.orientation, sim.apply_yaw(_yaw(p.orientation)))
    cov = list(msg.pose.covariance)
    k2 = sim.scale**2
    # Rotate the xy block and scale it; keep the rest.
    c, s = math.cos(sim.yaw), math.sin(sim.yaw)
    sxx, sxy, syy = cov[0], cov[1], cov[7]
    cov[0] = k2 * (c * c * sxx - 2 * c * s * sxy + s * s * syy)
    cov[7] = k2 * (s * s * sxx + 2 * c * s * sxy + c * c * syy)
    cov[1] = cov[6] = k2 * (c * s * (sxx - syy) + (c * c - s * s) * sxy)
    out.pose.covariance = cov
    return out


class LocalizationManager(Node):
    def __init__(self):
        super().__init__("localization_manager")
        self.maps_dir = Path(self.declare_parameter("maps_dir", "maps").value)
        self.db_dir = Path(self.declare_parameter("rtabmap_db_dir", "rtabmap_db").value)
        align_path = self.declare_parameter("alignments", "alignments.yaml").value
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_timeout = self.declare_parameter("odom_timeout", 0.5).value
        self.rtab_frame = self.declare_parameter("rtabmap_frame", "rtabmap_map").value
        self.alignments = load_alignments(align_path)
        self.floor = None
        self.mode_set = False
        self.last_odom_ok = -math.inf
        self.vo_was_ok = False
        self.reset_time = 0.0  # floor switch or visual odometry loss: needs a new fix
        self.last_fix = -math.inf

        pub = self.create_publisher
        self.indoor_pose_pub = pub(PoseWithCovarianceStamped, "guide/indoor_pose", 10)
        self.gps_pub = pub(Odometry, "guide/gps_odom", 10)
        self.imu_abs_pub = pub(Imu, "guide/imu_abs", 50)
        self.set_pose_pub = pub(PoseWithCovarianceStamped, "set_pose", 10)
        self.rtab_init_pub = pub(PoseWithCovarianceStamped, "rtabmap/initialpose", 10)
        self.localized_pub = pub(Bool, "guide/localized", 10)
        self.outdoor_pub = pub(Bool, "guide/outdoor", 10)

        sub = self.create_subscription
        sub(String, "guide/localization_mode", self.on_mode, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(Odometry, "vo", self.on_vo, 20)
        sub(Odometry, "odometry/gps", self.on_gps, 10)
        sub(Imu, "phone/imu", self.on_phone_imu, 50)
        sub(PoseWithCovarianceStamped, "rtabmap/localization_pose", self.on_rtab_pose, 10)
        sub(PoseWithCovarianceStamped, "guide/sign_fix", self.on_sign_fix, 10)
        sub(PoseWithCovarianceStamped, "guide/set_pose", self.on_set_pose, 10)

        from nav2_msgs.srv import LoadMap

        self.load_map = self.create_client(LoadMap, "map_server/load_map")
        try:
            from rtabmap_msgs.srv import LoadDatabase  # noqa: F401

            self.load_db = self.create_client(LoadDatabase, "rtabmap/load_database")
        except ImportError:
            self.load_db = None
        self.pending_map = None
        self.map_call = None
        self.create_timer(0.2, self.publish_localized)
        self.create_timer(1.0, self.send_map)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def outdoor(self) -> bool:
        return self.floor is None

    def on_mode(self, msg: String):
        try:
            floor = json.loads(msg.data).get("floor")
        except (ValueError, AttributeError):
            self.get_logger().error(f"bad localization_mode {msg.data!r}")
            return
        if self.mode_set and floor == self.floor:
            return
        self.mode_set = True
        self.floor = floor
        self.reset_time = self.now()
        self.outdoor_pub.publish(Bool(data=self.outdoor))
        self.get_logger().info(f"localization mode: {'outdoor' if self.outdoor else floor}")
        self.switch_map(floor)

    def switch_map(self, floor):
        grid = self.maps_dir / ("outdoor.yaml" if floor is None else f"floors/{floor}.yaml")
        if not grid.exists():
            self.get_logger().error(f"no grid {grid}; Nav2 keeps the previous map")
            self.pending_map = None
        else:
            self.pending_map = str(grid.resolve())
            self.send_map()
        if floor is not None and floor not in self.alignments:
            self.get_logger().warn(f"no RTAB-Map alignment for {floor}; run align_floor (spec section 6)")
        if self.load_db is not None and floor is not None:
            from rtabmap_msgs.srv import LoadDatabase

            db = self.db_dir / f"{floor}.db"
            if db.exists() and self.load_db.service_is_ready():
                req = LoadDatabase.Request()
                req.database_path = str(db.resolve())
                req.clear = False
                self.load_db.call_async(req)
            elif not db.exists():
                self.get_logger().warn(f"no RTAB-Map database {db}")

    def send_map(self):
        """Ask map_server for the pending grid; retried by a timer until it answers."""
        from nav2_msgs.srv import LoadMap

        if self.pending_map is None or self.map_call is not None:
            return
        if not self.load_map.service_is_ready():
            self.get_logger().info("waiting for map_server/load_map", throttle_duration_sec=10.0)
            return
        req = LoadMap.Request()
        req.map_url = self.pending_map
        url = self.pending_map
        self.map_call = self.load_map.call_async(req)

        def done(fut):
            self.map_call = None
            res = fut.result()
            if res is not None and res.result == LoadMap.Response.RESULT_SUCCESS:
                if self.pending_map == url:
                    self.pending_map = None
                self.get_logger().info(f"loaded {url}")
            else:
                self.get_logger().error(f"map_server could not load {url}; retrying")

        self.map_call.add_done_callback(done)

    def on_vo(self, msg: Odometry):
        if msg.pose.covariance[0] < LOST_COVARIANCE:
            self.last_odom_ok = self.now()

    def on_gps(self, msg: Odometry):
        if self.outdoor:
            self.gps_pub.publish(msg)

    def on_phone_imu(self, msg: Imu):
        if self.outdoor and msg.orientation_covariance[0] >= 0:
            self.imu_abs_pub.publish(msg)

    def on_rtab_pose(self, msg: PoseWithCovarianceStamped):
        sim = self.alignments.get(self.floor) if self.floor else None
        if sim is None:
            return
        self.last_fix = self.now()
        self.indoor_pose_pub.publish(transform_pose(msg, sim, self.map_frame))

    def on_sign_fix(self, msg: PoseWithCovarianceStamped):
        if self.outdoor:
            return
        self.last_fix = self.now()
        self.indoor_pose_pub.publish(msg)

    def on_set_pose(self, msg: PoseWithCovarianceStamped):
        self.set_pose_pub.publish(msg)
        sim = self.alignments.get(self.floor) if self.floor else None
        if sim is not None:
            self.rtab_init_pub.publish(transform_pose(msg, sim.inverse(), self.rtab_frame))

    def localized(self) -> bool:
        now = self.now()
        vo_ok = now - self.last_odom_ok <= self.odom_timeout
        if self.vo_was_ok and not vo_ok:
            # Odometry restarts from scratch after a loss; indoors that needs a new fix.
            self.reset_time = now
        self.vo_was_ok = vo_ok
        if not vo_ok:
            return False
        if self.outdoor:
            return True
        return self.last_fix >= self.reset_time

    def publish_localized(self):
        self.localized_pub.publish(Bool(data=self.localized()))
        if self.mode_set:
            # Re-sent so late starters (the sign reader, the gate) always know.
            self.outdoor_pub.publish(Bool(data=self.outdoor))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
