import numpy as np
from typing import Callable, Optional
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
    InteractiveMarkerFeedback,
)
from roboplan import Scene
from roboplan_ros_py.kinematics import RoboPlanIK
from roboplan_ros_py.type_conversions import se3_to_pose


class InteractiveMarkerIK:
    """
    Utility for creating an interactive marker in RViz and passing published
    Poses to an IK solver. Useful for nodes that ingest poses from users and
    publish or process IK solutions for those poses.
    """

    def __init__(
        self,
        node: Node,
        scene: Scene,
        ik_solver: RoboPlanIK,
        solved_callback: Optional[Callable[[Optional[np.ndarray], Pose], None]],
        namespace: str = "roboplan_ik",
    ):
        """
        Initialize the interactive marker IK controller.

        Args:
            node: ROS node instance to be passed to the InteractiveMarker server.
            scene: A fully configured RoboPlan scene.
            ik_solver: RoboPlanIK instance for solving IK.
            solved_callback: Function to be called each time an IK solution is found.
                             Requires signature `callback(joint_positions, pose)`,
                             where joint_positions is np.ndarray or None if IK failed.
            namespace: Namespace for the InteractiveMarkerServer.
        """
        self.node_ = node
        self.scene_ = scene
        self.ik_solver_ = ik_solver
        self.on_ik_solved_callback_ = solved_callback
        self.namespace_ = namespace

        # Set initial states based on the state of the scene
        self.last_joint_positions_ = self.scene_.getCurrentJointPositions()
        se3_pose = scene.forwardKinematics(
            self.last_joint_positions_, self.ik_solver_.tip_frame
        )
        self.current_pose_ = se3_to_pose(se3_pose)

        # Create the interactive marker server
        self.ik_server_ = InteractiveMarkerServer(self.node_, self.namespace_)
        self._create_interactive_marker(self.current_pose_)
        self.ik_server_.applyChanges()

    def _create_interactive_marker(self, pose: Pose):
        """
        Constructs a new interactive marker at the specified pose.
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.ik_solver_.base_frame
        int_marker.name = "ik_target"
        int_marker.description = f"IK Target Pose for {self.namespace_}"
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

        # Rotation controls
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

        # Add the marker to the server with the feedback callback
        self.ik_server_.insert(
            int_marker, feedback_callback=self._marker_feedback_callback
        )

    def _marker_feedback_callback(self, feedback: InteractiveMarkerFeedback):
        """
        Callback function to process pose updates from the iMarker.

        Args:
            feedback: InteractiveMarkerFeedback message
        """
        # Solve IK continuously as the marker is dragged
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.current_pose = feedback.pose
            self._solve_ik(feedback.pose)

    def _solve_ik(self, target_pose: Pose):
        """
        Solve IK for the target pose.

        Args:
            target_pose: The tarket pose for the IK solver.
        """
        joint_positions = self.ik_solver_.solve_ik(
            target_pose, seed_state=self.last_joint_positions_
        )

        if joint_positions is not None:
            self.last_joint_positions_ = joint_positions

        if self.on_ik_solved_callback_:
            self.on_ik_solved_callback_(joint_positions, target_pose)

    def get_current_pose(self):
        """Get the current pose of the interactive marker."""
        return self.current_pose_

    def get_last_joint_positions(self):
        """Get the last computed joint positions."""
        return self.last_joint_positions_

    def update_marker_pose(self, pose: Pose):
        """
        Update the marker pose.

        Args:
            pose: New pose for the marker
        """
        self.ik_server_.setPose("ik_target", pose)
        self.ik_server_.applyChanges()
        self.current_pose_ = pose
