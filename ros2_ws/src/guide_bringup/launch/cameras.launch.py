"""Stereo webcams, rectification, SGBM disparity and point cloud (spec sections 4 and 7).

Topics: /stereo/left/image_rect, /stereo/right/image_rect, /stereo/points2.
With ``mono:=true`` (spec section 18, one camera) only the left camera runs,
rectified by image_proc to the same /stereo/left/image_rect; no depth.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(__file__))
from common import Node, cfg  # noqa: E402


def cameras(context):
    calib = os.path.expanduser(LaunchConfiguration("calibration_dir").perform(context))
    mono = LaunchConfiguration("mono").perform(context).lower() == "true"
    nodes = []
    # camera_driver:=false: something else (the HIL simulator) publishes image_raw and camera_info.
    driver = LaunchConfiguration("camera_driver").perform(context).lower() == "true"
    for side in (("left",) if mono else ("left", "right")) if driver else ():
        nodes.append(Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name=f"{side}_camera",
            namespace=f"stereo/{side}",
            parameters=[cfg("cameras.yaml"), {"camera_info_url": f"file://{calib}/{side}.yaml"}],
        ))
    if mono:
        nodes.append(Node(package="image_proc", executable="rectify_node", name="left_rectify", namespace="stereo/left",
                          remappings=[("image", "image_raw"), ("image_rect", "image_rect")]))
    else:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("stereo_image_proc"), "launch", "stereo_image_proc.launch.py")),
            launch_arguments={
                "namespace": "stereo",
                "approximate_sync": "True",   # the webcams are not hardware synchronised
                "stereo_algorithm": "1",      # 1 = SGBM
                "disparity_range": "64",
                "correlation_window_size": "7",
                "use_color": "False",
            }.items(),
        ))
    if driver and LaunchConfiguration("rear").perform(context).lower() == "true":
        nodes.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="rear_camera", namespace="rear", parameters=[cfg("cameras.yaml")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_dir", default_value="~/.ros/camera_info", description="left.yaml and right.yaml from the stereo calibrator"),
        DeclareLaunchArgument("rear", default_value="false", description="phase 6 rear camera"),
        DeclareLaunchArgument("mono", default_value="false", description="one camera: left only, no depth"),
        DeclareLaunchArgument("camera_driver", default_value="true", description="false: image_raw comes from elsewhere (simulator)"),
        OpaqueFunction(function=cameras),
    ])
