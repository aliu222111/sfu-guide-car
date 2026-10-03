"""Phase 1: handheld stereo rig, build one floor's RTAB-Map database (spec section 11).

    ros2 launch guide_bringup mapping.launch.py floor:=AQ-03

Carry the rig down every corridor of the floor and close a loop. Then align it
to the GIS map with ``ros2 run guide_mission align_floor`` at three doors.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from common import repo_path  # noqa: E402

HERE = os.path.dirname(__file__)


def include(name, **args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, name)), launch_arguments=args.items())


def setup(context):
    floor = LaunchConfiguration("floor").perform(context)
    return [
        include("description.launch.py"),
        include("cameras.launch.py", calibration_dir=LaunchConfiguration("calibration_dir").perform(context)),
        include("rtabmap.launch.py", mode="mapping", handheld="true", database=repo_path("rtabmap_db", f"{floor}.db"),
                fresh=LaunchConfiguration("fresh").perform(context)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("floor", default_value="AQ-03"),
        DeclareLaunchArgument("calibration_dir", default_value="~/.ros/camera_info"),
        DeclareLaunchArgument("fresh", default_value="false"),
        OpaqueFunction(function=setup),
    ])
