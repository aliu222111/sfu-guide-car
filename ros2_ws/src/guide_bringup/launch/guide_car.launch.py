"""Everything on the car. Run from the repository root (maps/, models/, rtabmap_db/ live there).

    ros2 launch guide_bringup guide_car.launch.py start_floor:=AQ-03

Switches follow the phases in spec section 11:
    perception:=false   phase 2 driving tests before perception (the gate then
                        does not require the person detector)
    voice:=true         phase 6 voice agent (your Claude subscription: CLAUDE_CODE_OAUTH_TOKEN)
    rear_camera:=true   phase 6 student tracking
    rssi:=true          phase 6 Bluetooth follow check
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from common import ROSBRIDGE_PARAMS, cfg, repo_path  # noqa: E402

HERE = os.path.dirname(__file__)


def include(name, **args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, name)), launch_arguments=args.items())


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def setup(context):
    floor = LaunchConfiguration("start_floor").perform(context)
    guide = cfg("guide.yaml")
    graph = repo_path("maps", "route_graph.json")
    perception = flag(context, "perception")
    actions = [
        include("description.launch.py"),
        include("cameras.launch.py", calibration_dir=LaunchConfiguration("calibration_dir").perform(context),
                rear=str(flag(context, "rear_camera")).lower()),
        include("localization.launch.py", gps_port=LaunchConfiguration("gps_port").perform(context)),
        include("rtabmap.launch.py", mode="localization", database=repo_path("rtabmap_db", f"{floor or 'AQ-03'}.db")),
        include("navigation.launch.py", start_floor=floor),
        Node(package="guide_bridge", executable="serial_bridge", name="serial_bridge", parameters=[guide,
             {"port": LaunchConfiguration("motor_port").perform(context)}]),
        Node(package="guide_bridge", executable="velocity_gate", name="velocity_gate", parameters=[guide,
             {"require_person_detector": perception}]),
        Node(package="guide_mission", executable="mission", name="mission", parameters=[guide,
             {"graph": graph, "start_floor": floor, "voice_mode": flag(context, "voice_mode")}]),
        Node(package="guide_ui", executable="ui_server", name="ui_server", parameters=[guide, {"maps_dir": repo_path("maps")}]),
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge", parameters=[ROSBRIDGE_PARAMS]),
        Node(package="guide_perception", executable="vp_heading", name="vp_heading", parameters=[guide],
             remappings=[("image", "/stereo/left/image_rect"), ("camera_info", "/stereo/left/camera_info")]),
    ]
    if perception:
        left = [("image", "/stereo/left/image_rect"), ("camera_info", "/stereo/left/camera_info")]
        models = {"model": repo_path("models", "segformer-b0-ade.onnx"), "labels": repo_path("models", "segformer-b0-ade.labels.json")}
        actions += [
            Node(package="guide_perception", executable="floor_segmentation", name="floor_segmentation", parameters=[guide, models], remappings=left),
            Node(package="guide_perception", executable="person_detection", name="person_detection", parameters=[guide,
                 {"model": repo_path("models", "yolo11n.onnx")}], remappings=left),
            Node(package="guide_perception", executable="sign_reader", name="sign_reader", parameters=[guide], remappings=left),
        ]
    if flag(context, "rear_camera"):
        actions.append(Node(package="guide_perception", executable="student_tracker", name="student_tracker",
                            parameters=[guide, {"model": repo_path("models", "yolo11n.onnx")}],
                            remappings=[("image", "/rear/image_raw"), ("camera_info", "/rear/camera_info")]))
    if flag(context, "voice"):
        actions.append(Node(package="guide_voice", executable="voice_agent", name="voice_agent", parameters=[guide,
                            {"graph": graph, "piper_voice": repo_path("voices", "en_US-lessac-medium.onnx")}]))
    if flag(context, "rssi"):
        actions.append(Node(package="guide_ui", executable="phone_rssi", name="phone_rssi", parameters=[guide]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_floor", default_value="", description='e.g. AQ-03; "" = outdoors'),
        DeclareLaunchArgument("calibration_dir", default_value="~/.ros/camera_info"),
        DeclareLaunchArgument("motor_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("gps_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("voice", default_value="false"),
        DeclareLaunchArgument("voice_mode", default_value="false", description="no phone: ask 'still with me?' at turns"),
        DeclareLaunchArgument("rear_camera", default_value="false"),
        DeclareLaunchArgument("rssi", default_value="false"),
        OpaqueFunction(function=setup),
    ])
