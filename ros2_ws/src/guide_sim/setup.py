from glob import glob

from setuptools import find_packages, setup

package_name = "guide_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["arduino/*.h", "arduino/*.cpp"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Hardware-in-the-loop simulation of the guide car",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hil_sim = guide_sim.hil_node:main",
            "hil_trip = guide_sim.hil_trip:main",
            "hil_map = guide_sim.hil_map:main",
        ],
    },
)
