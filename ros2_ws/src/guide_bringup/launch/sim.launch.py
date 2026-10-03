"""The car's software with a kinematic stand-in for the hardware (no cameras, motors or GPS).

    ros2 launch guide_bringup sim.launch.py start_floor:=AQ-03 start:=entrance:aq_convocation_mall

Then open http://localhost:8080 and pick a room, or
    ros2 topic pub --once /guide/destination std_msgs/String "{data: 'AQ 3150'}"
"""

import json
import math
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


def setup(context):
    floor = LaunchConfiguration("start_floor").perform(context)
    start = LaunchConfiguration("start").perform(context)
    graph_path = repo_path("maps", "route_graph.json")
    with open(graph_path) as f:
        nodes = {n["id"]: n for n in json.load(f)["nodes"]}
    if start not in nodes:
        raise RuntimeError(f"unknown start node {start}")
    n = nodes[start]
    guide = cfg("guide.yaml")
    return [
        Node(package="guide_bringup", executable="fake_robot", name="fake_robot",
             parameters=[{"x": n["x"], "y": n["y"], "yaw": float(LaunchConfiguration("yaw").perform(context)), "indoor": bool(floor)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, "navigation.launch.py")),
                                 launch_arguments={"start_floor": floor}.items()),
        Node(package="guide_bridge", executable="velocity_gate", name="velocity_gate", parameters=[guide]),
        Node(package="guide_mission", executable="mission", name="mission",
             parameters=[guide, {"graph": graph_path, "start_floor": floor}]),
        Node(package="guide_mission", executable="localization_manager", name="localization_manager",
             parameters=[guide, {"maps_dir": repo_path("maps"), "rtabmap_db_dir": repo_path("rtabmap_db"),
                                 "alignments": repo_path("rtabmap_db", "alignments.yaml")}]),
        Node(package="guide_ui", executable="ui_server", name="ui_server", parameters=[guide, {"maps_dir": repo_path("maps")}]),
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge", parameters=[ROSBRIDGE_PARAMS]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_floor", default_value="AQ-03"),
        DeclareLaunchArgument("start", default_value="entrance:aq_convocation_mall"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=setup),
    ])
