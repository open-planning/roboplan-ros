import numpy as np
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import roboplan


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


def to_joint_trajectory(roboplan_trajectory: roboplan.JointTrajectory) -> JointTrajectory:
    """
    Converts a roboplan.JointTrajectory object to a ROS 2 JointTrajectory message.

    Args:
        roboplan_trajectory: The roboplan JointTrajectory to convert.

    Returns:
        An equivalent ROS 2 JointTrajectory message.
    """
    ros_traj = JointTrajectory()
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


def from_joint_trajectory(ros_trajectory: JointTrajectory) -> roboplan.JointTrajectory:
    """
    Convert the provided ROS 2 JointTrajectory message to an equivalent
    roboplan.JointTrajectory.

    Args:
        ros_trajectory: The ROS 2 JointTrajectory message to convert.

    Returns:
        An equivalent roboplan.JointTrajectory.
    """
    joint_traj = roboplan.JointTrajectory()

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
