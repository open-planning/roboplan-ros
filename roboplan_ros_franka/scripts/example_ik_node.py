#!/usr/bin/env python3

import os
import sys
import xacro
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from roboplan import Scene, SimpleIkOptions
from roboplan_ros_py.kinematics import RoboPlanIK
from roboplan_ros_py.interactive_marker_ik import InteractiveMarkerIK


class InteractiveMarkerIKNode(Node):
    """
    ROS 2 node that provides an interactive marker for IK-based pose control.
    Publishes joint states when IK solutions are found.
    """

    def __init__(self):
        super().__init__("interactive_marker_ik_node")

        # Hardcoded Franka FR3 configuration
        self.joint_group = "fr3_arm"
        self.base_frame = "fr3_link0"
        self.tip_frame = "fr3_hand"

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

        # Start in a reasonable pose
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

        # Get joint group information
        joint_group_info = self.scene.getJointGroupInfo(self.joint_group)
        self.q_indices = joint_group_info.q_indices
        joint_names = np.array(self.scene.getJointNames())
        self.joint_names = joint_names[self.q_indices]

        # Publish joint states as we know them at a minimum of 10 Hz
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Set up the IK solver
        self.get_logger().info(f"Setting up IK solver for group: {self.joint_group}")
        options = SimpleIkOptions()
        options.group_name = self.joint_group
        options.max_iters = 100
        options.step_size = 0.25
        self.ik_solver = RoboPlanIK(
            scene=self.scene,
            group_name=self.joint_group,
            base_frame=self.base_frame,
            tip_frame=self.tip_frame,
            options=options,
        )

        # Create the interactive marker controller
        self.get_logger().info("Creating interactive marker controller...")
        self.imarker_ik = InteractiveMarkerIK(
            node=self,
            scene=self.scene,
            ik_solver=self.ik_solver,
            solved_callback=self.on_ik_solved,
            namespace="franka_ik_marker",
        )

        self.get_logger().info("Interactive Marker IK Node initialized successfully!")
        self.get_logger().info(f"Joint group: {self.joint_group}")
        self.get_logger().info(f"Joints: {self.joint_names}")
        self.get_logger().info(f"End-effector: {self.tip_frame}")
        self.get_logger().info(
            "Move the interactive marker in RViz to generate IK solutions"
        )

    def on_ik_solved(self, joint_positions: np.ndarray, _: Pose):
        """
        Callback function called when IK is solved.

        Args:
            joint_positions: Solved joint positions (None if IK failed)
            pose: Target pose that was solved for
        """
        if joint_positions is not None:
            # Expand to full positions for publishing
            self.latest_joint_positions[self.q_indices] = joint_positions
            self.publish_joint_states()
        else:
            self.get_logger().warn(
                "IK failed to find solution for target pose", throttle_duration_sec=1.0
            )

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "world"
        joint_state_msg.name = self.scene.getJointNames()
        joint_state_msg.position = self.latest_joint_positions.tolist()
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerIKNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        sys.exit(1)
    finally:
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
