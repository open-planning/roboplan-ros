#!/usr/bin/env python3

import os
import threading
from collections.abc import Callable
from typing import Optional
import numpy as np

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)
from roboplan.core import Scene
from roboplan.simple_ik import SimpleIkOptions
from roboplan_ros_py.kinematics import RoboPlanIK
from roboplan_ros_cpp.bindings import poseToSE3, se3ToPose


class InteractiveMarkerIK:
    """
    Utility for creating an interactive marker in RViz and passing published
    poses to an IK solver. Useful for nodes that ingest poses from users and
    publish or process IK solutions for those poses.
    """

    def __init__(
        self,
        node: Node,
        scene: Scene,
        joint_group: str,
        base_link: str,
        tip_link: str,
        options: SimpleIkOptions = SimpleIkOptions(),
        solve_callback: Optional[Callable] = None,
        node_name: str = "imarker_ik_node",
        namespace: str = "/roboplan_ik",
    ):
        """
        Initialize the interactive marker IK controller.

        Args:
            node: ROS node instance to be passed to the InteractiveMarker server.
            scene: A fully configured RoboPlan scene.
            joint_group: The joint group for the IK solver
            base_link: Base link of the IK chain.
            tip_link: Tip link of the iK chain.
            options: Options for the IK solver.
            solve_callback: Callback function to immediately handle IK solutions.
            node_name: Name for the dedicated Node for callbacks to the IK marker.
            namespace: Namespace for the InteractiveMarkerServer and JointStatePublisher.
        """
        self.node = node
        self.scene = scene
        self.joint_group = joint_group
        self.base_link = base_link
        self.tip_link = tip_link
        self.namespace = namespace
        self.solve_callback = solve_callback

        # Get joint group information
        joint_group_info = self.scene.getJointGroupInfo(self.joint_group)
        self._q_indices = joint_group_info.q_indices
        joint_names = np.array(self.scene.getJointNames())
        self._joint_names = joint_names[self._q_indices]

        # Construct the IK solver
        self._ik_solver = RoboPlanIK(
            scene=self.scene,
            group_name=self.joint_group,
            base_frame=self.base_link,
            tip_frame=self.tip_link,
            options=options,
        )

        # Set initial states based on the state of the scene
        self.last_joint_positions = self.scene.getCurrentJointPositions()
        se3_pose = scene.forwardKinematics(self.last_joint_positions, self.tip_link)
        self._current_pose = se3ToPose(se3_pose)
        self._target_pose = self._current_pose

        # Create a separate node for the marker server with its own executor. There are likely
        # better solutions but in the interim this removes all latency to let updates be processed
        # as soon as they come in, otherwise the server seems to get hung up.
        self._marker_node = Node(node_name)
        self._ik_server = InteractiveMarkerServer(self._marker_node, self.namespace)
        self._create_interactive_marker(self._current_pose)
        self._ik_server.applyChanges()

        self._marker_executor = SingleThreadedExecutor()
        self._marker_executor.add_node(self._marker_node)
        self._marker_thread = threading.Thread(
            target=self._spin_marker_executor, daemon=True
        )
        self._marker_thread.start()

        # Create the namespaced joint state publisher and message
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._joint_state_pub = self.node.create_publisher(
            JointState, os.path.join(self.namespace, "joint_states"), qos
        )

        self._joint_state_msg = JointState()
        self._joint_state_msg.name = self.scene.getJointNames()
        self._joint_state_msg.header.frame_id = ""

        self.node.get_logger().info("Constructed interactive marker controller")
        self.node.get_logger().info(f"Joint group: {self.joint_group}")
        self.node.get_logger().info(f"Joints: {self._joint_names}")
        self.node.get_logger().info(f"Base link: {self.base_link}")
        self.node.get_logger().info(f"End-effector: {self.tip_link}")

    def _spin_marker_executor(self):
        try:
            self._marker_executor.spin()
        except ExternalShutdownException:
            pass

    def shutdown(self):
        self._marker_executor.shutdown()
        self._marker_thread.join(timeout=0.25)
        self._marker_node.destroy_node()

    def _create_interactive_marker(self, pose: Pose):
        """
        Constructs a new interactive marker at the specified pose.
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_link
        int_marker.name = "ik_target"
        int_marker.description = f"IK Target Pose for {self.joint_group}"
        int_marker.pose = pose
        int_marker.scale = 0.2

        sphere_marker = Marker()
        sphere_marker.type = Marker.SPHERE
        sphere_marker.scale.x = 0.025
        sphere_marker.scale.y = 0.025
        sphere_marker.scale.z = 0.025
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.5
        sphere_marker.color.b = 1.0
        sphere_marker.color.a = 1.0

        sphere_control = InteractiveMarkerControl()
        sphere_control.always_visible = True
        int_marker.controls.append(sphere_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        self._ik_server.insert(
            int_marker, feedback_callback=self._marker_feedback_callback
        )

    def _marker_feedback_callback(self, feedback: InteractiveMarkerFeedback):
        """
        Callback function to process pose updates from the iMarker.

        Args:
            feedback: InteractiveMarkerFeedback message
        """
        # Record the iMarker state as it is updated and solve
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._target_pose = feedback.pose
            self.solve_ik()

    def solve_ik(self):
        """
        Solve IK for the latest set target pose.
        """
        # Get the transform for the target pose and solve
        transform = poseToSE3(self._target_pose)
        joint_positions = self._ik_solver.solve_ik(
            transform, seed_state=self.last_joint_positions
        )

        if joint_positions is not None:
            self.last_joint_positions = joint_positions
            if self.solve_callback:
                self.solve_callback(joint_positions)
        else:
            self.node.get_logger().warn("IK failed to find solution for target pose")

    def publish_joint_states(self):
        """
        Publish the latest solution's joint states.
        """
        self._joint_state_msg.header.stamp = self.node.get_clock().now().to_msg()
        self._joint_state_msg.position = self.last_joint_positions.tolist()
        self._joint_state_pub.publish(self._joint_state_msg)
