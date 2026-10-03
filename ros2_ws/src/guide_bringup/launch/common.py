"""Helpers shared by the launch files (not a launch file itself)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as _Node
from launch_ros.parameter_descriptions import ParameterValue

SHARE = get_package_share_directory("guide_bringup")


# Sim time for the hardware-in-the-loop simulator (guide_sim): the launch
# argument use_sim_time (default false) is every node's last parameter layer.
# Not launch_ros SetParameter: a global parameter makes rcl drop dict
# overrides of keys that guide.yaml also sets (motor port, indoor_max).
SIM_TIME = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time", default="false"), value_type=bool)}


def Node(**kw):  # noqa: N802 (drop-in for launch_ros.actions.Node)
    kw["parameters"] = list(kw.get("parameters") or []) + [SIM_TIME]
    return _Node(**kw)


def cfg(name: str) -> str:
    return os.path.join(SHARE, "config", name)


def repo_path(*parts: str) -> str:
    """Paths like maps/ and models/ resolve against the repo root (the launch's working directory)."""
    return os.path.abspath(os.path.join(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()), *parts))


def nav2_params(src: str, cameras: str) -> str:
    """nav2.yaml for the camera setup. Stereo: as written. One camera (spec
    section 18): no depth cloud, so the voxel layer is off and floor segmentation
    is the costmaps' only obstacle source. iPhone (spec section 19): the depth
    cloud is the LiDAR's. Written next to the other launch temp files; returns its path."""
    import tempfile

    import yaml

    if cameras == "stereo":
        return src
    with open(src) as f:
        p = yaml.safe_load(f)
    local = p["local_costmap"]["local_costmap"]["ros__parameters"]
    obstacle = p["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    if cameras == "mono":
        local["plugins"] = [x for x in local["plugins"] if x != "voxel_layer"]
        local.pop("voxel_layer", None)
        obstacle["observation_sources"] = "floor_obstacles"
        obstacle.pop("stereo", None)
    else:
        local["voxel_layer"]["stereo"]["topic"] = "/front/points"
        obstacle["stereo"]["topic"] = "/front/points"
    fd, path = tempfile.mkstemp(prefix=f"nav2_{cameras}_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(p, f)
    return path


# The phone app reaches ROS through rosbridge on campus WiFi, so anyone on the
# network can connect. Allow only the app's own topics: no motor, enable or
# localization topics, no services, no actions.
ROSBRIDGE_PARAMS = {
    "port": 9090,
    "topics_pub_glob": "['/guide/destination', '/guide/confirm', '/guide/pause', '/guide/cancel', '/guide/heartbeat']",
    "topics_sub_glob": "['/guide/state', '/guide/speech', '/guide/route']",
    "services_glob": "[]",
    "actions_glob": "[]",
}
