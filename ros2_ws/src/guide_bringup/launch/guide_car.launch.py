"""Everything on the car. Run from the repository root (maps/, models/, rtabmap_db/ live there).

    ros2 launch guide_bringup guide_car.launch.py start_floor:=AQ-03

Switches follow the phases in spec section 11:
    perception:=false   phase 2 driving tests before perception (the gate then
                        does not require the person detector)
    voice:=true         phase 6 voice agent (ELEVENLABS_API_KEY and GEMINI_API_KEY; Claude only with llm: claude)
    rear_camera:=true   phase 6 student tracking
    rssi:=true          phase 6 Bluetooth follow check
    cameras:=mono       one camera (spec section 18): left camera only, floor
                        odometry instead of RTAB-Map, no depth cloud
    cameras:=iphone     the LiDAR iPhone in front (spec section 19): ARKit odometry,
                        LiDAR depth, RTAB-Map in RGB-D mode; add rear_camera:=true
                        for the webcam facing the student

For the hardware-in-the-loop simulator (guide_sim hil.launch.py):
    camera_driver:=false  the simulator publishes the cameras' image_raw
    record3d_path:=...    the simulator's stand-in for the iPhone's USB stream
    sim_perception:=true  the simulator stands in for the model-based perception
                          nodes (people, signs, floor); the gate still requires them
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(__file__))
from common import Node, ROSBRIDGE_PARAMS, cfg, repo_path  # noqa: E402

HERE = os.path.dirname(__file__)


def include(name, **args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, name)), launch_arguments=args.items())


def flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def setup(context):
    floor = LaunchConfiguration("start_floor").perform(context)
    guide = cfg("guide.yaml")
    graph = repo_path("maps", "route_graph.json")
    sim_perception = flag(context, "sim_perception")
    perception = flag(context, "perception") and not sim_perception
    cams = LaunchConfiguration("cameras").perform(context)
    mono = cams == "mono"
    image = "/front" if cams == "iphone" else "/stereo/left"
    left = [("image", f"{image}/image_rect"), ("camera_info", f"{image}/camera_info")]
    if mono:
        odometry = Node(package="guide_perception", executable="mono_odometry", name="mono_odometry",
                        parameters=[guide], remappings=left + [("vo", "/vo")])
    else:  # the iPhone driver publishes /vo itself; RTAB-Map localizes on its colour and depth
        odometry = include("rtabmap.launch.py", mode="localization", database=repo_path("rtabmap_db", f"{floor or 'AQ-03'}.db"),
                           sensor="rgbd" if cams == "iphone" else "stereo")
    actions = [
        include("description.launch.py"),
        include("cameras.launch.py", calibration_dir=LaunchConfiguration("calibration_dir").perform(context),
                rear=str(flag(context, "rear_camera")).lower(), cameras=cams,
                camera_driver=str(flag(context, "camera_driver")).lower(),
                record3d_path=LaunchConfiguration("record3d_path").perform(context)),
        include("localization.launch.py", gps_port=LaunchConfiguration("gps_port").perform(context), cameras=cams),
        odometry,
        include("navigation.launch.py", start_floor=floor, cameras=cams,
                depth_period=LaunchConfiguration("depth_period").perform(context)),
        Node(package="guide_bridge", executable="serial_bridge", name="serial_bridge", parameters=[guide,
             {"port": LaunchConfiguration("motor_port").perform(context)}]),
        Node(package="guide_bridge", executable="velocity_gate", name="velocity_gate", parameters=[guide,
             {"require_person_detector": perception or sim_perception,
              "require_bumper": mono},     # one camera: nothing else sees glass or anything above the floor
             {"indoor_max": 0.4} if mono else {}]),
        Node(package="guide_mission", executable="mission", name="mission", parameters=[guide,
             {"graph": graph, "start_floor": floor, "voice_mode": flag(context, "voice_mode")}]),
        Node(package="guide_ui", executable="ui_server", name="ui_server", parameters=[guide, {"maps_dir": repo_path("maps")}]),
        Node(package="guide_api", executable="scout_api", name="scout_api", parameters=[guide, {"maps_dir": repo_path("maps")}]),
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge", parameters=[ROSBRIDGE_PARAMS]),
        Node(package="guide_perception", executable="vp_heading", name="vp_heading", parameters=[guide],
             remappings=left),
    ]
    if perception:
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
                            remappings=[("image", "/rear/image_rect"), ("camera_info", "/rear/camera_info")]))
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
        # Both are USB serial adapters: find them under /dev/serial/by-id/ (README, "The RC car").
        DeclareLaunchArgument("motor_port", default_value="/dev/ttyUSB0", description="the ESP32 (CP2102 / CH340)"),
        DeclareLaunchArgument("gps_port", default_value="/dev/ttyUSB1", description="the Adafruit GPS adapter"),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("cameras", default_value="stereo", choices=["stereo", "mono", "iphone"]),
        DeclareLaunchArgument("camera_driver", default_value="true"),
        DeclareLaunchArgument("record3d_path", default_value=""),
        DeclareLaunchArgument("depth_period", default_value="", description="simulator only: max age of the depth cloud, s"),
        DeclareLaunchArgument("sim_perception", default_value="false"),
        DeclareLaunchArgument("voice", default_value="false"),
        DeclareLaunchArgument("voice_mode", default_value="false", description="no phone: ask 'still with me?' at turns"),
        DeclareLaunchArgument("rear_camera", default_value="false"),
        DeclareLaunchArgument("rssi", default_value="false"),
        OpaqueFunction(function=setup),
    ])
