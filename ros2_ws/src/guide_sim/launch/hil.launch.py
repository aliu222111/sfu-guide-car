"""The whole car in simulation: real firmware, real nodes, simulated hardware and building.

    ros2 launch guide_sim hil.launch.py                       # stereo, RTAB-Map (needs rtabmap_db/<floor>.db)
    ros2 launch guide_sim hil.launch.py cameras:=mono         # one camera, no database needed
    ros2 launch guide_sim hil.launch.py cameras:=iphone       # LiDAR iPhone (needs its own database, as stereo)
    ros2 run guide_sim hil_trip "AQ 3046"                     # the student, through rosbridge like the phone app

Stereo needs the floor's RTAB-Map database and alignment, made as in phase 1:
    ros2 launch guide_sim hil.launch.py mode:=map rig_destination:="AQ 3046" sim_root:=/tmp/guide_sim_root
    ros2 run guide_sim hil_map --floor AQ-03 --root /tmp/guide_sim_root
    ros2 launch guide_sim hil.launch.py sim_root:=/tmp/guide_sim_root      # then hil_trip as above

Run from the repository root. The simulator owns /clock; every other node runs
on sim time. guide_car.launch.py is the launch file the car uses, with the
serial bridge and GPS driver pointed at the simulator's pseudo serial ports
and usb_cam replaced by the simulator's images.
"""

import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import guide_sim

LINKS = "/tmp/guide_sim"
# The iPhone driver loads this stand-in record3d library, which reads the simulator's stream.
RECORD3D = os.path.join(os.path.dirname(guide_sim.__file__), "fake_record3d")


def sim_root(repo: str, root: str) -> str:
    """A scratch root for the car's launch files: maps/ linked, its own rtabmap_db/."""
    os.makedirs(os.path.join(root, "rtabmap_db"), exist_ok=True)
    link = os.path.join(root, "maps")
    if not os.path.exists(link):
        os.symlink(os.path.join(repo, "maps"), link)
    return root


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in
         ("cameras", "start", "max_rtf", "clutter", "people", "seed", "camera_rate", "turn_slip", "mode",
          "rig_destination", "sim_root")}
    repo = os.environ.get("GUIDE_CAR_ROOT", os.getcwd())
    nodes = {n["id"]: n for n in json.load(open(os.path.join(repo, "maps", "route_graph.json")))["nodes"]}
    floor = nodes[a["start"]]["floor"]
    if a["sim_root"]:
        # The car's launch files find maps/ and rtabmap_db/ through GUIDE_CAR_ROOT (common.repo_path).
        os.environ["GUIDE_CAR_ROOT"] = sim_root(repo, a["sim_root"])
    mapping = a["mode"] == "map"
    if mapping and not a["rig_destination"]:
        raise RuntimeError("mode:=map needs rig_destination:=<room>")
    sim = Node(package="guide_sim", executable="hil_sim", name="hil_sim", output="screen", parameters=[{
        "use_sim_time": False,  # it is the clock
        "start": a["start"], "cameras": a["cameras"], "max_rtf": float(a["max_rtf"]),
        "clutter": int(a["clutter"]), "people": int(a["people"]), "seed": int(a["seed"]),
        "camera_rate": float(a["camera_rate"]), "turn_slip": float(a["turn_slip"]), "link_dir": LINKS,
        "maps_dir": os.path.join(repo, "maps"),
        "firmware": os.path.join(repo, "ros2_ws", "src", "guide_bridge", "firmware", "guide_motor", "guide_motor.ino"),
        "rig_destination": a["rig_destination"] if mapping else "",
    }])
    share = get_package_share_directory("guide_bringup")
    if mapping:
        # Phase 1: the handheld rig. Description (camera TF), rectification and
        # RTAB-Map in mapping mode, on sim time; nothing of the car.
        inc = lambda name, **kw: IncludeLaunchDescription(  # noqa: E731
            PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
            launch_arguments={"use_sim_time": "true", **kw}.items())
        db = os.path.join(os.environ.get("GUIDE_CAR_ROOT", repo), "rtabmap_db", f"{floor}.db")
        iphone = a["cameras"] == "iphone"
        rig = [inc("description.launch.py"),
               inc("cameras.launch.py", camera_driver="false", cameras=a["cameras"], iphone_tf="true", record3d_path=RECORD3D),
               inc("rtabmap.launch.py", mode="mapping", handheld="true", fresh="true", database=db,
                   sensor="rgbd" if iphone else "stereo")]
        return [sim, TimerAction(period=4.0, actions=rig)]
    car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "guide_car.launch.py")),
        launch_arguments={
            "start_floor": floor, "cameras": a["cameras"], "camera_driver": "false", "sim_perception": "true",
            "motor_port": f"{LINKS}/motor", "gps_port": f"{LINKS}/gps", "record3d_path": RECORD3D,
            "use_sim_time": "true",  # every node of the car (guide_bringup common.Node)
        }.items(),
    )
    # The car starts once the simulator has made its serial ports and is publishing /clock.
    return [sim, TimerAction(period=4.0, actions=[car])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cameras", default_value="stereo", choices=["stereo", "mono", "iphone"]),
        DeclareLaunchArgument("start", default_value="entrance:aq_convocation_mall", description="route graph node"),
        DeclareLaunchArgument("max_rtf", default_value="0.5", description="cap on simulated / real time"),
        DeclareLaunchArgument("clutter", default_value="6", description="unmapped boxes on the start floor"),
        DeclareLaunchArgument("people", default_value="3", description="people walking the start floor"),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("camera_rate", default_value="10.0"),
        DeclareLaunchArgument("turn_slip", default_value="1.45", description="tank: geometric / actual turn rate"),
        DeclareLaunchArgument("mode", default_value="trip", choices=["trip", "map"]),
        DeclareLaunchArgument("rig_destination", default_value="", description="mode:=map: map the route to this room and back"),
        DeclareLaunchArgument("sim_root", default_value="", description="scratch root with maps/ and rtabmap_db/ (stereo)"),
        OpaqueFunction(function=setup),
    ])
