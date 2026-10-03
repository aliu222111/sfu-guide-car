"""Stereo visual odometry and RTAB-Map (spec section 6).

sensor:=stereo     the webcam pair; stereo odometry publishes /vo.
sensor:=rgbd       the iPhone (spec section 19): its driver publishes /vo (ARKit)
                   and RTAB-Map uses its colour image and LiDAR depth.

mode:=mapping      one database per floor, built on a drive of each corridor (phase 1).
mode:=localization against that database; publishes rtabmap/localization_pose,
                   which the localization manager aligns to the GIS frame.

RTAB-Map does not publish map -> odom here: the global EKF owns that transform.
In mapping mode on the handheld rig (``handheld:=true``) there is no EKF, so
odometry and RTAB-Map publish TF themselves.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from common import Node  # noqa: E402


def nodes(context):
    mode = LaunchConfiguration("mode").perform(context)
    handheld = LaunchConfiguration("handheld").perform(context).lower() == "true"
    odom_imu = LaunchConfiguration("odom_imu").perform(context).lower() == "true"
    rgbd = LaunchConfiguration("sensor").perform(context) == "rgbd"
    db = os.path.abspath(os.path.expanduser(LaunchConfiguration("database").perform(context)))
    os.makedirs(os.path.dirname(db), exist_ok=True)
    if rgbd:
        sensor_remaps = [("rgb/image", "/front/image_rect"), ("depth/image", "/front/depth"),
                         ("rgb/camera_info", "/front/camera_info")]
    else:
        sensor_remaps = [
            ("left/image_rect", "/stereo/left/image_rect"),
            ("right/image_rect", "/stereo/right/image_rect"),
            ("left/camera_info", "/stereo/left/camera_info"),
            ("right/camera_info", "/stereo/right/camera_info"),
        ]
    common = {
        "frame_id": "base_link",
        "approx_sync": True,           # unsynchronised webcams (spec section 7)
        "approx_sync_max_interval": 0.02,
        "wait_imu_to_init": False,
    }
    odom = Node(
        package="rtabmap_odom",
        executable="stereo_odometry",
        name="stereo_odometry",
        parameters=[common, {
            "publish_tf": handheld,
            "odom_frame_id": "odom",
            "Odom/Strategy": "0",          # frame-to-map
            "Vis/MinInliers": "15",
            "Odom/ResetCountdown": "0",    # stay lost (and stopped) rather than silently reset
        }],
        # The phone gyro reaches the EKF directly. Feeding it to stereo odometry too
        # is opt-in: in the simulator an IMU without orientation warped the first
        # corner of every lap (3.4 m worst error against 0.65 m without it).
        remappings=sensor_remaps + [("odom", "/vo"), ("imu", "/phone/imu" if odom_imu else "imu_unused")],
    )
    localization = mode == "localization"
    slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        arguments=[] if localization else ["--delete_db_on_start"] if LaunchConfiguration("fresh").perform(context).lower() == "true" else [],
        parameters=[common, {
            "subscribe_stereo": not rgbd,
            "subscribe_depth": rgbd,
            "subscribe_rgb": False,
            "database_path": db,
            "map_frame_id": "map" if handheld else "rtabmap_map",
            # No odom_frame_id here: with it set, RTAB-Map ignores the odometry topic
            # and reads odom -> base_link from TF at each image's stamp, which fails
            # for every frame stereo odometry skipped. Subscribing to the topic pairs
            # each image with the odometry computed from it.
            "publish_tf": handheld,
            # odom_sensor_sync re-reads odom -> base_link at each image's stamp. On the
            # car that is the 30 Hz EKF. On the handheld rig it is stereo odometry,
            # which skips frames when slower than the cameras, so the lookup would
            # fail and RTAB-Map would drop almost every frame.
            "odom_sensor_sync": not handheld,
            "Mem/IncrementalMemory": "false" if localization else "true",
            "Mem/InitWMWithAllNodes": "true" if localization else "false",
            "RGBD/OptimizeMaxError": "3.0",
            "Reg/Force3DoF": "true",       # the car stays on one floor plane
        }],
        remappings=sensor_remaps + [("odom", "/vo" if handheld else "/odometry/local")],
    )
    return [slam] if rgbd else [odom, slam]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="localization", choices=["mapping", "localization"]),
        DeclareLaunchArgument("database", default_value="rtabmap_db/AQ-03.db"),
        DeclareLaunchArgument("handheld", default_value="false"),
        DeclareLaunchArgument("sensor", default_value="stereo", choices=["stereo", "rgbd"]),
        DeclareLaunchArgument("odom_imu", default_value="false", description="give stereo odometry the phone IMU"),
        DeclareLaunchArgument("fresh", default_value="false", description="mapping: start a new database"),
        OpaqueFunction(function=nodes),
    ])
