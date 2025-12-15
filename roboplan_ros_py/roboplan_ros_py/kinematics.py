import numpy as np

from typing import Optional


from roboplan.core import Scene, JointConfiguration, CartesianConfiguration
from roboplan.simple_ik import SimpleIkOptions, SimpleIk


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

        A thin wrapper around the RoboPlan SimpleIK class that manages joint states and group
        indices. Callers can expect to use the full set of joint states for the Scene.

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

        # Initialize the IK solver and configurations
        self._ik_solver = SimpleIk(self.scene, self.options)
        self._start = JointConfiguration()
        self._goal = CartesianConfiguration()
        self._goal.base_frame = self.base_frame
        self._goal.tip_frame = self.tip_frame

        # Get joint group information
        self.joint_group_info_ = scene.getJointGroupInfo(group_name)
        self._q_indices = self.joint_group_info_.q_indices

    def solve_ik(
        self,
        target_transform: np.ndarray,
        seed_state: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve IK for a target transform.

        Args:
            target_transform: A pin.SE3 4x4 transformation matrix.
            seed_state: Optional seed joint positions, defaults to current pose.

        Returns:
            A set of joint positions for the desired pose, or else None.
        """
        cur_pos_full = np.array(self.scene.getCurrentJointPositions())
        if seed_state is None:
            seed_state = cur_pos_full[self._q_indices]
        else:
            if len(seed_state) == len(cur_pos_full):
                seed_state = seed_state[self._q_indices]

        # Setup start and goal configurations
        self._start.positions = seed_state
        self._goal.tform = target_transform

        # Solve
        solution = JointConfiguration()
        result = self._ik_solver.solveIk(self._goal, self._start, solution)
        if result:
            cur_pos_full[self._q_indices] = solution.positions
            return cur_pos_full
        else:
            return None
