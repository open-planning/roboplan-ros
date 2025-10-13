"""
Example launch file for a Franka arm using ros2_control mock hardware.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("roboplan_example_models"),
        "models",
        "franka_robot_model",
        "fr3.urdf",
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath, mappings={},
    ).toprettyxml(indent="  ")

    controllers_yaml_filepath = os.path.join(
        get_package_share_directory("roboplan_ros_franka"),
        "config",
        "ros2_controllers.yaml",
    )
    rviz_config_filepath = os.path.join(
        get_package_share_directory("roboplan_ros_franka"),
        "config",
        "franka_config.rviz",
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                controllers_yaml_filepath,
                {"robot_description": robot_description},
            ],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["fr3_arm_controller"],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_filepath],
        )
    ]
    return LaunchDescription(nodes)
