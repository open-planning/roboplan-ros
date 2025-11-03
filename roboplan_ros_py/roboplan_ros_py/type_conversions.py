import numpy as np
import pinocchio as pin

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory as ROSJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from roboplan import (
    CartesianConfiguration,
    JointConfiguration,
    JointTrajectory,
)


def to_duration(time_sec: float) -> Duration:
    """
    Converts a double timestamp to an equivalent ROS Duration.

    Args:
        time_sec: Timestamp in seconds.

    Returns:
        ROS 2 Duration with seconds and nanoseconds.
    """
    duration = Duration()
    duration.sec = int(time_sec)
    duration.nanosec = int((time_sec - duration.sec) * 1e9)
    return duration


def from_duration(duration: Duration) -> float:
    """
    Converts a ROS 2 Duration with seconds and nanoseconds to an equivalent double timestamp.

    Args:
        duration: ROS 2 Duration with seconds and nanoseconds.

    Returns:
        An equivalent double timestamp.
    """
    return duration.sec + duration.nanosec * 1e-9


def to_joint_state(joint_configuration: JointConfiguration) -> JointState:
    """
    Converts a roboplan.JointConfiguration object to a ROS 2 JointState message.

    Args:
        joint_configuration: The roboplan JointConfiguration to convert.

    Returns:
        An equivalent ROS 2 JointState message
    """
    joint_state = JointState()
    joint_state.name = joint_configuration.joint_names
    joint_state.position = joint_configuration.positions.tolist()
    joint_state.velocity = joint_configuration.velocities.tolist()

    # Effort != acceleration but including and kind of abusing the interface.
    joint_state.effort = joint_configuration.accelerations.tolist()
    return joint_state


def from_joint_state(joint_state: JointState):
    """
    Convert the provided ROS 2 JointState message to an equivalent
    roboplan.JointConfiguration.

    Args:
        joint_state: The ROS 2 JointState message to convert.

    Returns:
        An equivalent roboplan.JointConfiguration.
    """
    joint_configuration = JointConfiguration()
    joint_configuration.joint_names = joint_state.name
    joint_configuration.positions = np.array(joint_state.position)
    joint_configuration.velocities = np.array(joint_state.velocity)

    # Effort != acceleration but including and kind of abusing the interface.
    joint_configuration.accelerations = np.array(joint_state.effort)
    return joint_configuration


def to_joint_trajectory(
    roboplan_trajectory: JointTrajectory,
) -> ROSJointTrajectory:
    """
    Converts a roboplan.JointTrajectory object to a ROS 2 JointTrajectory message.

    Args:
        roboplan_trajectory: The roboplan JointTrajectory to convert.

    Returns:
        An equivalent ROS 2 JointTrajectory message.
    """
    ros_traj = ROSJointTrajectory()
    ros_traj.joint_names = roboplan_trajectory.joint_names

    ros_traj.points = []
    for i in range(len(roboplan_trajectory.times)):
        point = JointTrajectoryPoint()
        time_sec = roboplan_trajectory.times[i]
        point.time_from_start = to_duration(time_sec)

        if i < len(roboplan_trajectory.positions):
            point.positions = roboplan_trajectory.positions[i].tolist()

        if i < len(roboplan_trajectory.velocities):
            point.velocities = roboplan_trajectory.velocities[i].tolist()

        if i < len(roboplan_trajectory.accelerations):
            point.accelerations = roboplan_trajectory.accelerations[i].tolist()

        ros_traj.points.append(point)

    return ros_traj


def from_joint_trajectory(ros_trajectory: ROSJointTrajectory) -> JointTrajectory:
    """
    Convert the provided ROS 2 JointTrajectory message to an equivalent
    roboplan.JointTrajectory.

    Args:
        ros_trajectory: The ROS 2 JointTrajectory message to convert.

    Returns:
        An equivalent roboplan.JointTrajectory.
    """
    joint_traj = JointTrajectory()

    joint_traj.joint_names = ros_trajectory.joint_names

    times = []
    positions = []
    velocities = []
    accelerations = []

    for point in ros_trajectory.points:
        times.append(from_duration(point.time_from_start))

        if point.positions:
            positions.append(np.array(point.positions))

        if point.velocities:
            velocities.append(np.array(point.velocities))

        if point.accelerations:
            accelerations.append(np.array(point.accelerations))

    joint_traj.times = times
    joint_traj.positions = positions
    joint_traj.velocities = velocities
    joint_traj.accelerations = accelerations

    return joint_traj


def to_transform_stamped(
    cartesian_configuration: CartesianConfiguration,
) -> TransformStamped:
    """
    Converts a roboplan.CartesianConfiguration to ROS TransformStamped.

    Args:
        cartesian_configuration: The roboplan.CartesianConfiguration to convert.

    Returns:
        An equivalent ROS TransformStamped message.
    """
    transform_stamped = TransformStamped()
    transform_stamped.header.frame_id = cartesian_configuration.base_frame
    transform_stamped.child_frame_id = cartesian_configuration.tip_frame

    transform_stamped.transform.translation.x = float(
        cartesian_configuration.tform[0, 3]
    )
    transform_stamped.transform.translation.y = float(
        cartesian_configuration.tform[1, 3]
    )
    transform_stamped.transform.translation.z = float(
        cartesian_configuration.tform[2, 3]
    )

    rotation = cartesian_configuration.tform[:3, :3]
    quat = pin.Quaternion(rotation)
    transform_stamped.transform.rotation.w = float(quat.w)
    transform_stamped.transform.rotation.x = float(quat.x)
    transform_stamped.transform.rotation.y = float(quat.y)
    transform_stamped.transform.rotation.z = float(quat.z)

    return transform_stamped


def from_transform_stamped(
    transform_stamped: TransformStamped,
) -> CartesianConfiguration:
    """
    Convert ROS TransformStamped to a roboplan.CartesianConfiguration.

    Args:
        transform_stamped: The ROS TransformStamped message to convert.

    Returns:
        An equivalent roboplan.CartesianConfiguration.
    """
    config = CartesianConfiguration()
    config.base_frame = transform_stamped.header.frame_id
    config.tip_frame = transform_stamped.child_frame_id

    position = np.array(
        [
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z,
        ]
    )
    quaternion = pin.Quaternion(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
    )
    config.tform = pin.SE3(quaternion, position).homogeneous

    return config


def pose_to_se3(pose: Pose) -> np.ndarray:
    """
    Convert ROS Pose to a pin.SE3 4x4 transformation matrix.

    Args:
        pose: ROS Pose message

    Returns:
        4x4 numpy array
    """
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    quaternion = pin.Quaternion(
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
    )
    return pin.SE3(quaternion, position).homogeneous


def se3_to_pose(transform: np.ndarray) -> Pose:
    """
    Convert a pin.SE3 4x4 transformation matrix to ROS Pose.

    Args:
        transform: 4x4 numpy array

    Returns:
        An equivalent ROS Pose message
    """
    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    rotation = transform[:3, :3]
    quat = pin.Quaternion(rotation)
    pose.orientation.w = float(quat.w)
    pose.orientation.x = float(quat.x)
    pose.orientation.y = float(quat.y)
    pose.orientation.z = float(quat.z)
    return pose
