import numpy as np

from typing import Optional
from geometry_msgs.msg import Pose

from roboplan_ros_py.type_conversions import pose_to_se3

from roboplan import (
    Scene,
    JointConfiguration,
    CartesianConfiguration,
    SimpleIkOptions,
    SimpleIk,
)


class RoboPlanIK:
    """
    IK solver wrapper for roboplan Scene objects with ROS message support.

    This class provides methods to solve IK given ROS Pose messages and
    returns results as JointState messages or numpy arrays.
    """

    def __init__(
        self,
        scene: Scene,
        group_name: str,
        base_frame: str,
        tip_frame: str,
        options: SimpleIkOptions,
    ):
        """
        Construct an IK solver.

        Args:
            scene: RoboPlan Scene object.
            group_name: Name of the joint group to use.
            base_frame: Base frame for IK.
            tip_frame: End effector/tip frame for IK.
            options: Options for the IK solver.
        """
        self.scene = scene
        self.group_name = group_name
        self.base_frame = base_frame
        self.tip_frame = tip_frame
        self.options = options

        # Initialize the IK solver
        self.ik_solver_ = SimpleIk(self.scene, self.options)

        # Get joint group information
        self.joint_group_info_ = scene.getJointGroupInfo(group_name)
        self.q_indices_ = self.joint_group_info_.q_indices

    def solve_ik(
        self,
        target_pose: Pose,
        seed_state: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve IK for a target pose.

        Args:
            target_pose: ROS Pose message with target position and orientation.
            seed_state: Optional seed joint positions, defaults to current pose.

        Returns:
            A set of joint positions for the desired pose, or else None.
        """
        cur_pos_full = np.array(self.scene.getCurrentJointPositions())
        if seed_state is None:
            seed_state = cur_pos_full[self.q_indices_]
        else:
            if len(seed_state) == len(cur_pos_full):
                seed_state = seed_state[self.q_indices_]

        target_transform = pose_to_se3(target_pose)

        # Setup start and goal configurations
        start = JointConfiguration()
        start.positions = seed_state

        goal = CartesianConfiguration()
        goal.base_frame = self.base_frame
        goal.tip_frame = self.tip_frame
        goal.tform = target_transform

        # Solve
        solution = JointConfiguration()
        result = self.ik_solver_.solveIk(goal, start, solution)
        if result:
            return solution.positions
        else:
            return None
