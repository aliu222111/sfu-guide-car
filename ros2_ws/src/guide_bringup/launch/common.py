"""Helpers shared by the launch files (not a launch file itself)."""

import os

from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory("guide_bringup")


def cfg(name: str) -> str:
    return os.path.join(SHARE, "config", name)


def repo_path(*parts: str) -> str:
    """Paths like maps/ and models/ resolve against the repo root (the launch's working directory)."""
    return os.path.abspath(os.path.join(os.environ.get("GUIDE_CAR_ROOT", os.getcwd()), *parts))
