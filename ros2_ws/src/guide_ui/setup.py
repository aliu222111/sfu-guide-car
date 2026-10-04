from glob import glob

from setuptools import find_packages, setup

package_name = "guide_ui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"guide_ui": ["web/*"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", glob("guide_ui/web/*")),
    ],
    install_requires=["setuptools"],  # bleak for phone_rssi (phase 6)
    zip_safe=False,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Phone web app server",
    license="MIT",
    entry_points={"console_scripts": ["ui_server = guide_ui.server_node:main", "phone_rssi = guide_ui.rssi_node:main"]},
)
