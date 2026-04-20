"""RoboPlan ROS Python bindings for C++ Type Conversions"""

from typing import Annotated

import numpy
from numpy.typing import NDArray


def se3ToPose(transform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> object:
    """Converts an SE3 transformation matrix to a geometry_msgs::msg::Pose."""

def poseToSE3(py_pose: object) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
    """Converts the provided geometry_msgs::msg::Pose to an SE3 Matrix."""

def toJointTrajectory(py_roboplan_traj: object) -> object:
    """
    Converts a roboplan::JointTrajectory to a trajectory_msgs::msg::JointTrajectory.
    """

def fromJointTrajectory(py_ros_traj: object) -> object:
    """
    Converts a trajectory_msgs::msg::JointTrajectory to a roboplan::JointTrajectory.
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

def fromJointState(py_joint_state: object, py_scene: object, conversion_map: JointStateConverterMap) -> "roboplan::JointConfiguration":
    """
    Converts a ROS 2 sensor_msgs::msg::JointState message to a roboplan::JointConfiguration.
    """
