"""The front camera or cameras, and the rear one (spec sections 4, 7, 18 and 19).

cameras:=stereo  two webcams: /stereo/left/image_rect, /stereo/right/image_rect,
                 SGBM disparity, /stereo/points2
cameras:=mono    one webcam (spec section 18): /stereo/left/image_rect only, no depth
cameras:=iphone  the LiDAR iPhone over USB (spec section 19): /front/image_rect,
                 /front/depth, /front/points and ARKit odometry on /vo
rear:=true       a webcam facing the student: /rear/image_rect (with cameras:=iphone
                 this is the one webcam)
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
    front = LaunchConfiguration("cameras").perform(context)
    mono = front == "mono"
    nodes = []
    # camera_driver:=false: something else (the HIL simulator) publishes image_raw and camera_info,
    # or, for the iPhone, streams in place of the phone (record3d_path: its stand-in library).
    driver = LaunchConfiguration("camera_driver").perform(context).lower() == "true"
    if front == "iphone":
        lib = LaunchConfiguration("record3d_path").perform(context)
        env = {"PYTHONPATH": f"{lib}:{os.environ.get('PYTHONPATH', '')}"} if lib else {}
        nodes.append(Node(package="guide_bridge", executable="iphone", name="iphone", additional_env=env,
                          parameters=[cfg("guide.yaml"),
                                      {"publish_tf": LaunchConfiguration("iphone_tf").perform(context).lower() == "true"}]))
    for side in () if front == "iphone" or not driver else ("left",) if mono else ("left", "right"):
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
    elif front == "stereo":
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
    if LaunchConfiguration("rear").perform(context).lower() == "true":
        if driver:
            nodes.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="rear_camera", namespace="rear",
                              parameters=[cfg("cameras.yaml"), {"camera_info_url": f"file://{calib}/rear.yaml"}]))
        # The tracker projects pixels with the rectified camera model: give it rectified images.
        nodes.append(Node(package="image_proc", executable="rectify_node", name="rear_rectify", namespace="rear",
                          remappings=[("image", "image_raw")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_dir", default_value="~/.ros/camera_info", description="left.yaml and right.yaml from the stereo calibrator"),
        DeclareLaunchArgument("rear", default_value="false", description="phase 6 rear camera"),
        DeclareLaunchArgument("cameras", default_value="stereo", choices=["stereo", "mono", "iphone"]),
        DeclareLaunchArgument("iphone_tf", default_value="false", description="iPhone odometry publishes odom -> base_link (handheld mapping)"),
        DeclareLaunchArgument("record3d_path", default_value="", description="simulator: directory of the stand-in record3d library"),
        DeclareLaunchArgument("camera_driver", default_value="true", description="false: image_raw comes from elsewhere (simulator)"),
        OpaqueFunction(function=cameras),
    ])
