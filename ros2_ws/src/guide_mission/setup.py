from setuptools import find_packages, setup

package_name = "guide_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # sfu_map is installed from the repo with pip (pip install -e sfu_map).
    install_requires=["setuptools", "sfu_map", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Route graph walker and trip state machine, localization mode manager",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mission = guide_mission.mission_node:main",
            "localization_manager = guide_mission.localization_manager_node:main",
            "align_floor = guide_mission.alignment:main",
        ],
    },
)
