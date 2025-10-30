"""
Launches the mock hardware alongside the sample Roboplan IK solution rendering node.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package_dir = get_package_share_directory("roboplan_ros_franka")

    rviz_config_filepath = os.path.join(
        package_dir,
        "config",
        "franka_ik_config.rviz",
    )

    # Include the "hw" launch file
    roboplan_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "franka_example.launch.py"),
        ),
        launch_arguments={
            "rviz_config": rviz_config_filepath,
        }.items(),
    )

    nodes = [
        Node(
            package="roboplan_ros_franka",
            executable="example_ik_node",
            name="example_ik_node",
            output="screen",
        ),
    ]

    return LaunchDescription([roboplan_control] + nodes)
