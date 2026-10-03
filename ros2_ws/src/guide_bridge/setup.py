from setuptools import find_packages, setup

package_name = "guide_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Serial link to the motor microcontroller, phone IMU bridge, velocity safety gate",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_bridge = guide_bridge.serial_bridge_node:main",
            "phone_imu = guide_bridge.phone_imu_node:main",
            "velocity_gate = guide_bridge.velocity_gate_node:main",
        ],
    },
)
