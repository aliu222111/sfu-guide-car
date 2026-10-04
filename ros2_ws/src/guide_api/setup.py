from setuptools import find_packages, setup

package_name = "guide_api"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"guide_api": ["web/*", "config/*.json"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Laird",
    maintainer_email="lairdscabar@gmail.com",
    description="HTTP API for the Scout rider app",
    license="MIT",
    entry_points={"console_scripts": ["scout_api = guide_api.api_node:main"]},
)
