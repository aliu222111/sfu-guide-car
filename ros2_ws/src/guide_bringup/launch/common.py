"""Helpers shared by the launch files (not a launch file itself)."""

import os

from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory("guide_bringup")


def cfg(name: str) -> str:
    return os.path.join(SHARE, "config", name)


def repo_path(*parts: str) -> str:
    """Paths like maps/ and models/ resolve against the repo root (the launch's working directory)."""
    return os.path.abspath(os.path.join(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()), *parts))


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
