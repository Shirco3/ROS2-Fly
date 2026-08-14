"""Launch the simulator, controller, and optional RViz2 visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    controller_share = get_package_share_directory("fly_catcher_controller")
    simulator_share = get_package_share_directory("fly_catcher_simulator")

    parameters_file = os.path.join(controller_share, "config", "controller.yaml")
    rviz_file = os.path.join(controller_share, "rviz", "fly_catcher.rviz")
    simulator_launch = os.path.join(simulator_share, "launch", "sim.launch.py")

    use_simulator = LaunchConfiguration("use_simulator")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_simulator",
                default_value="true",
                description="Start the provided fly and drone simulator",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the supplied marker configuration",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch),
                condition=IfCondition(use_simulator),
            ),
            Node(
                package="fly_catcher_controller",
                executable="controller",
                name="fly_catcher_controller",
                output="screen",
                parameters=[parameters_file],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fly_catcher_rviz",
                output="screen",
                arguments=["-d", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
