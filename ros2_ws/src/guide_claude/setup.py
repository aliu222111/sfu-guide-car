from setuptools import find_packages, setup

package_name = "guide_claude"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "claude-agent-sdk"],
    zip_safe=True,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Claude through the owner's Claude subscription (Claude Agent SDK)",
    license="MIT",
)
