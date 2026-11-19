"""RoboPlan ROS nanobind wrappers for core C++ ROS tools."""

from typing import Annotated, overload

import numpy
from numpy.typing import NDArray


def se3ToPose(transform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> object:
    """Converts an SE3 transformation matrix to a geometry_msgs::msg::Pose."""

def poseToSE3(py_pose: object) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
    """Converts the provided geometry_msgs::msg::Pose to an SE3 Matrix."""

def toJointTrajectory(py_roboplan_traj: object, py_scene: "roboplan::Scene") -> object:
    """
    Converts a roboplan::JointTrajectory to a trajectory_msgs::msg::JointTrajectory. Continuous joints become angles, unwrapped along the trajectory, and mimic joints are left out. Planar and floating joints are an error: use toRosTrajectories for them.
    """

def toRosTrajectories(py_roboplan_traj: object, py_scene: "roboplan::Scene") -> tuple:
    """
    Converts a roboplan::JointTrajectory with any kind of joint to a (trajectory_msgs::msg::JointTrajectory, trajectory_msgs::msg::MultiDOFJointTrajectory) tuple: the single degree of freedom joints as in toJointTrajectory, the planar and floating joints as transforms, with twists in each joint's own frame.
    """

@overload
def fromJointTrajectory(py_ros_traj: object, py_scene: "roboplan::Scene") -> "roboplan::JointTrajectory":
    """
    Converts a trajectory_msgs::msg::JointTrajectory to a roboplan::JointTrajectory. Continuous joints become [cos, sin] positions, and the values of mimic joints are ignored. Planar and floating joints are an error: use the overload that also takes a MultiDOFJointTrajectory.
    """

@overload
def fromJointTrajectory(py_ros_traj: object, py_multi_dof_traj: object, py_scene: "roboplan::Scene") -> "roboplan::JointTrajectory":
    """
    Converts a trajectory_msgs::msg::JointTrajectory with the single degree of freedom joints and a trajectory_msgs::msg::MultiDOFJointTrajectory with the planar and floating joints to one roboplan::JointTrajectory, with the joints of the JointTrajectory in their order, then those of the MultiDOFJointTrajectory in theirs.
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
