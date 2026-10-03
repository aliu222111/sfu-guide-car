"""Robot description: static transforms for base_link, cameras, phone and GPS."""

import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("guide_bringup"), "urdf", "guide_car.urdf.xacro")
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}],
        ),
    ])
