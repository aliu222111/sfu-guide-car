"""Helpers shared by the launch files (not a launch file itself)."""

import os

from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory("guide_bringup")


def cfg(name: str) -> str:
    return os.path.join(SHARE, "config", name)


def repo_path(*parts: str) -> str:
    """Paths like maps/ and models/ resolve against the repo root (the launch's working directory)."""
    return os.path.abspath(os.path.join(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()), *parts))


def mono_nav2_params(src: str) -> str:
    """nav2.yaml for one camera (spec section 18): no stereo point cloud, so the
    voxel layer is off and floor segmentation is the costmaps' only obstacle
    source. Written next to the other launch temp files; returns its path."""
    import tempfile

    import yaml

    with open(src) as f:
        p = yaml.safe_load(f)
    local = p["local_costmap"]["local_costmap"]["ros__parameters"]
    local["plugins"] = [x for x in local["plugins"] if x != "voxel_layer"]
    local.pop("voxel_layer", None)
    obstacle = p["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    obstacle["observation_sources"] = "floor_obstacles"
    obstacle.pop("stereo", None)
    fd, path = tempfile.mkstemp(prefix="nav2_mono_", suffix=".yaml")
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
