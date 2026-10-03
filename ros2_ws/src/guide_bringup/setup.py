from glob import glob

from setuptools import setup

package_name = "guide_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=["guide_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
        ("share/" + package_name + "/scripts", glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["fake_robot = guide_bringup.fake_robot:main", "sim_trip = guide_bringup.sim_trip:main", "stereo_sim = guide_bringup.stereo_sim:main", "rtabmap_check = guide_bringup.rtabmap_check:main"]},
    zip_safe=True,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Launch files, robot description and parameters for the SFU guide car",
    license="MIT",
)
