from setuptools import setup
import os
from glob import glob

package_name = "aruco_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("lib", package_name), ["scripts/detection_node", "scripts/debug_viewer", "scripts/yaw_test_node", "scripts/attitude_node"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    scripts=["scripts/detection_node", "scripts/debug_viewer", "scripts/yaw_test_node", "scripts/attitude_node"],
)
