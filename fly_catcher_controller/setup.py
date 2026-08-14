import os
from glob import glob

from setuptools import find_packages, setup

package_name = "fly_catcher_controller"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Shir ben dror",
    maintainer_email="shir2861999@gmail.com",
    description="Lead/intercept ROS 2 controller for the Fly Catcher assignment.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = fly_catcher_controller.controller:main",
        ],
    },
)
