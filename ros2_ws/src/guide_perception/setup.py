from setuptools import find_packages, setup

package_name = "guide_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # Model runtimes are optional per node: onnxruntime (floor), ultralytics
    # (people), paddleocr or anthropic (signs). See requirements-perception.txt.
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Alex Liu",
    maintainer_email="alexthespacexfan@gmail.com",
    description="Floor segmentation, person detection, sign reading and corridor vanishing-point heading",
    license="MIT",
    entry_points={
        "console_scripts": [
            "floor_segmentation = guide_perception.floor_seg_node:main",
            "person_detection = guide_perception.person_node:main",
            "sign_reader = guide_perception.sign_node:main",
            "vp_heading = guide_perception.vp_heading_node:main",
            "student_tracker = guide_perception.student_node:main",
            "mono_odometry = guide_perception.mono_odometry_node:main",
        ],
    },
)
