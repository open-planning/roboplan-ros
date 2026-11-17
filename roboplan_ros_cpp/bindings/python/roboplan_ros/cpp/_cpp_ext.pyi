"""RoboPlan ROS nanobind wrappers for core C++ ROS tools."""

from typing import Annotated

import numpy
from numpy.typing import NDArray


def se3ToPose(transform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> object:
    """Converts an SE3 transformation matrix to a geometry_msgs::msg::Pose."""

def poseToSE3(py_pose: object) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
    """Converts the provided geometry_msgs::msg::Pose to an SE3 Matrix."""

def toJointTrajectory(py_roboplan_traj: object, py_scene: object | None = None) -> object:
    """
    Converts a roboplan::JointTrajectory to a trajectory_msgs::msg::JointTrajectory. With a scene, the [cos, sin] positions of continuous joints become angles, unwrapped along the trajectory, and mimic, planar and floating joints are left out; without one, positions are copied as they are.
    """

def toMultiDOFJointTrajectory(py_roboplan_traj: object, py_scene: object) -> object:
    """
    Converts the planar and floating joints of a roboplan::JointTrajectory to a trajectory_msgs::msg::MultiDOFJointTrajectory, with twists in each joint's own frame.
    """

def fromJointTrajectory(py_ros_traj: object, py_scene: object | None = None, py_multi_dof_traj: object | None = None) -> object:
    """
    Converts a trajectory_msgs::msg::JointTrajectory, and optionally a trajectory_msgs::msg::MultiDOFJointTrajectory with the planar and floating joints, to a roboplan::JointTrajectory. With a scene, the angles of continuous joints become [cos, sin] positions and the values of mimic joints are ignored; without one, positions are copied as they are.
    """

def toTransformStamped(cartesian_config: "roboplan::CartesianConfiguration") -> object:
    """
    Converts a roboplan::CartesianConfiguration to a geometry_msgs::msg::TransformStamped.
    """

def fromTransformStamped(py_transform: object) -> object:
    """
    Converts a geometry_msgs::msg::TransformStamped to a roboplan::CartesianConfiguration.
    """

class JointMapping:
    @property
    def joint_name(self) -> str: ...

    @property
    def ros_index(self) -> int: ...

    @property
    def q_start(self) -> int: ...

    @property
    def v_start(self) -> int: ...

    @property
    def type(self) -> "roboplan::JointType": ...

class JointStateConverterMap:
    @property
    def mappings(self) -> list[JointMapping]: ...

    @property
    def nq(self) -> int: ...

    @property
    def nv(self) -> int: ...

def buildConversionMap(py_scene: object, py_joint_state: object) -> JointStateConverterMap:
    """
    Constructs a JointState conversion map given a RoboPlan Scene and JointState message.
    """

def toJointState(py_config: object, py_scene: object) -> object:
    """
    Converts a roboplan::JointConfiguration to a ROS 2 sensor_msgs::msg::JointState message.
    """

def fromJointState(py_joint_state: object, py_scene: object, conversion_map: JointStateConverterMap, q_reference: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None = None) -> "roboplan::JointConfiguration":
    """
    Converts a ROS 2 sensor_msgs::msg::JointState message to a roboplan::JointConfiguration. Joints that are not in the message keep their positions from q_reference, or from the scene's current state if it is not given.
    """
