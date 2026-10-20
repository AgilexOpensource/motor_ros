import os
from glob import glob

from setuptools import setup

package_name = "motor_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Agilex Robotics",
    maintainer_email="contact@agilex.ai",
    description="Generic ROS 2 bridge for agx_motor_sdk ArmJointGroup modules",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motor_node = motor_ros.motor_node:main",
        ],
    },
)
