import os
from glob import glob

from setuptools import find_packages, setup

package_name = "smabo_brain_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smabo",
    maintainer_email="daigaku.robot@gmail.com",
    description="ROS 2 wrapper around smabo-brain (reuses brain pure logic).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_node = smabo_brain_ros.odom_node:main",
            "servo_trajectory_bridge = smabo_brain_ros.servo_trajectory_bridge:main",
            "webrtc_camera_node = smabo_brain_ros.webrtc_camera_node:main",
            "speech_recognizer_node = smabo_brain_ros.speech_recognizer_node:main",
            "image_processor_node = smabo_brain_ros.image_processor_node:main",
            "gaze_policy_node = smabo_brain_ros.gaze_policy_node:main",
            "neck_policy_node = smabo_brain_ros.neck_policy_node:main",
            "drive_policy_node = smabo_brain_ros.drive_policy_node:main",
        ],
    },
)
