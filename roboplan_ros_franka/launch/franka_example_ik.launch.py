"""
Example launch file for a Franka arm using ros2_control mock hardware.
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    models_dir = os.path.join(
        get_package_share_directory("roboplan_example_models"),
        "models",
        "franka_robot_model",
    )

    franka_urdf_filepath = os.path.join(models_dir, "fr3.urdf")
    franka_srdf_filepath = os.path.join(models_dir, "fr3.srdf")
    franka_yaml_filepath = os.path.join(models_dir, "fr3_config.yaml")

    robot_description = xacro.process_file(
        franka_urdf_filepath,
        mappings={},
    ).toprettyxml(indent="  ")
    robot_description_semantic = xacro.process_file(
        franka_srdf_filepath,
        mappings={},
    ).toprettyxml(indent="  ")

    rviz_config_filepath = os.path.join(
        get_package_share_directory("roboplan_ros_franka"),
        "config",
        "franka_ik_config.rviz",
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="roboplan_ros_visualization",
            executable="interactive_marker_ik_node",
            name="interactive_marker_ik_node_franka",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    "robot_description_package": "roboplan_example_models",
                    "yaml_config_path": franka_yaml_filepath,
                    "joint_group": "fr3_arm",
                    "base_frame": "fr3_link0",
                    "tip_frame": "fr3_hand",
                    "marker_namespace": "franka_ik_marker",
                    # Reasonable start pose...
                    "initial_joint_positions": [
                        0.0,
                        -0.785,
                        0.0,
                        -2.356,
                        0.0,
                        1.571,
                        0.785,
                        0.0,
                        0.0,
                    ],
                    "max_ik_iterations": 100,
                    "ik_step_size": 0.25,
                    "joint_state_publish_rate": 10.0,
                }
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_filepath],
        ),
    ]
    return LaunchDescription(nodes)
