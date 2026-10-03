"""EKFs, navsat_transform, GPS driver, phone IMU bridge, localization manager (spec section 6)."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from common import Node, cfg, repo_path  # noqa: E402


def generate_launch_description():
    ekf = cfg("ekf.yaml")
    guide = cfg("guide.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("gps_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("cameras", default_value="stereo", choices=["stereo", "mono"]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_local", parameters=[ekf],
             remappings=[("odometry/filtered", "/odometry/local"), ("set_pose", "/ekf_local/set_pose")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_global", parameters=[ekf],
             remappings=[("odometry/filtered", "/odometry/global"), ("set_pose", "/set_pose")]),
        Node(package="robot_localization", executable="navsat_transform_node", name="navsat_transform", parameters=[ekf],
             remappings=[("imu", "/guide/imu_abs"), ("gps/fix", "/gps/fix"), ("odometry/filtered", "/odometry/global"),
                         ("odometry/gps", "/odometry/gps")]),
        # Adafruit Ultimate GPS (MTK3339) on a USB serial adapter, 10 Hz after configuring it.
        # respawn: the driver exits if the port is missing (GPS unplugged, or enumerated late).
        Node(package="nmea_navsat_driver", executable="nmea_serial_driver", name="gps", respawn=True, respawn_delay=3.0,
             parameters=[{"port": LaunchConfiguration("gps_port"), "baud": 9600, "frame_id": "gps_link"}],
             remappings=[("fix", "/gps/fix")]),
        Node(package="guide_bridge", executable="phone_imu", name="phone_imu", parameters=[guide]),
        Node(package="guide_mission", executable="localization_manager", name="localization_manager",
             parameters=[guide, {"maps_dir": repo_path("maps"), "rtabmap_db_dir": repo_path("rtabmap_db"),
                                 "alignments": repo_path("rtabmap_db", "alignments.yaml"),
                                 "use_rtabmap": ParameterValue(PythonExpression(["'", LaunchConfiguration("cameras"), "' != 'mono'"]),
                                                               value_type=bool)}]),
    ])
