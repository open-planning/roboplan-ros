#!/usr/bin/env python3

import os
import sys
import xacro
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import ColorRGBA

from roboplan import Scene, SimpleIkOptions
from roboplan_ros_visualization.interactive_marker_ik import InteractiveMarkerIK
from roboplan_ros_visualization.roboplan_visualizer import RoboplanVisualizer


class InteractiveMarkerIKNode(Node):
    """
    ROS 2 node that provides an interactive marker for IK-based pose control.
    Publishes joint states when IK solutions are found.
    """

    def __init__(self):
        super().__init__("interactive_marker_ik_node")

        # Hardcoded Franka FR3 configuration
        self.joint_group = "fr3_arm"
        self.base_link = "fr3_link0"
        self.tip_link = "fr3_hand"

        # Get robot description files
        models_dir = os.path.join(
            get_package_share_directory("roboplan_example_models"),
            "models",
            "franka_robot_model",
        )

        urdf_path = os.path.join(models_dir, "fr3.urdf")
        srdf_path = os.path.join(models_dir, "fr3.srdf")
        yaml_config_path = os.path.join(models_dir, "fr3_config.yaml")

        # Process files with xacro
        self.get_logger().info("Processing URDF and SRDF with xacro...")
        urdf_xml = xacro.process_file(urdf_path).toxml()
        srdf_xml = xacro.process_file(srdf_path).toxml()

        # Create the RoboPlan scene
        self.get_logger().info("Creating RoboPlan scene...")
        package_paths = [get_package_share_directory("roboplan_example_models")]

        self.scene = Scene(
            name="franka_ik_scene",
            urdf=urdf_xml,
            srdf=srdf_xml,
            package_paths=package_paths,
            yaml_config_path=yaml_config_path,
        )

        # Start in a reasonable pose, this could come from hardware.
        self.latest_joint_positions = np.array(
            [
                0.0,
                -0.785,
                0.0,
                -2.356,
                0.0,
                1.571,
                0.785,
                0,
                0,
            ]
        )
        self.scene.setJointPositions(self.latest_joint_positions)

        # Set the IK solver options
        options = SimpleIkOptions()
        options.group_name = self.joint_group
        options.max_iters = 100
        options.step_size = 0.25

        # Setup a visualize marker model for rendering IK solutions
        self.ik_viz = RoboplanVisualizer(
            node=self,
            scene=self.scene,
            urdf_xml=urdf_xml,
            package_paths=package_paths,
            color=ColorRGBA(r=0.5, g=0.5, b=1.0, a=0.5),
            namespace="roboplan_ik",
        )

        # Create the interactive marker controller, which will call the viz to
        # update anytime a solution is found.
        self.get_logger().info("Creating interactive marker controller...")
        self.imarker_ik = InteractiveMarkerIK(
            node=self,
            scene=self.scene,
            joint_group=self.joint_group,
            base_link=self.base_link,
            tip_link=self.tip_link,
            options=options,
            solve_callback=self.ik_viz.visualize_configuration,
            namespace="roboplan_ik",
        )

        self.get_logger().info(
            "Move the interactive marker in RViz to visualize IK solutions"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerIKNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
