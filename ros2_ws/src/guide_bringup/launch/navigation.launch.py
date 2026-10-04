"""Nav2 servers (spec section 8). The controller's output goes to cmd_vel_nav, through the
velocity gate, then to the motors. No AMCL and no velocity smoother: robot_localization
provides map -> odom and the gate limits acceleration."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from common import Node, SHARE, cfg, nav2_params, repo_path  # noqa: E402

SERVERS = ["map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"]


def nodes(context):
    params = nav2_params(cfg("nav2.yaml"), LaunchConfiguration("cameras").perform(context),
                         LaunchConfiguration("depth_period").perform(context))
    floor = LaunchConfiguration("start_floor").perform(context)
    grid = repo_path("maps", "outdoor.yaml" if not floor else f"floors/{floor}.yaml")
    bt = os.path.join(SHARE, "behavior_trees", "navigate_through_poses.xml")
    to_nav = [("cmd_vel", "/cmd_vel_nav")]
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server", parameters=[params, {"yaml_filename": grid}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", parameters=[params]),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", parameters=[params], remappings=to_nav),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", parameters=[params], remappings=to_nav),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             parameters=[params, {"default_nav_through_poses_bt_xml": bt}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             parameters=[{"autostart": True, "node_names": SERVERS, "bond_timeout": 10.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_floor", default_value="", description='floor id such as AQ-03, or "" for outdoors'),
        DeclareLaunchArgument("cameras", default_value="stereo", choices=["stereo", "mono", "iphone"]),
        DeclareLaunchArgument("depth_period", default_value="", description="simulator only: max age of the depth cloud, s"),
        OpaqueFunction(function=nodes),
    ])
