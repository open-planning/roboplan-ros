import pytest
import numpy as np
import roboplan
from roboplan_ros_py.type_conversions import (
    to_joint_trajectory,
    from_joint_trajectory,
)


def test_empty_conversions():
    roboplan_traj = roboplan.JointTrajectory()
    ros_traj = to_joint_trajectory(roboplan_traj)
    assert len(ros_traj.joint_names) == 0

    orig_roboplan_traj = from_joint_trajectory(ros_traj)
    assert len(orig_roboplan_traj.joint_names) == 0


def test_convert_to_joint_trajectory():
    # Create a roboplan trajectory with two joints and two points, convert it to/from
    # ROS and ensure we get the same data back.
    roboplan_traj = roboplan.JointTrajectory()
    roboplan_traj.joint_names = ["joint1", "joint2"]
    roboplan_traj.times = [0.0, 1.0]
    roboplan_traj.positions = [np.array([0.0, 0.0]), np.array([1.0, -1.0])]
    roboplan_traj.velocities = [np.array([0.0, 0.0]), np.array([2.0, -2.0])]
    roboplan_traj.accelerations = [np.array([0.0, 0.0]), np.array([3.0, -3.0])]

    ros_traj = to_joint_trajectory(roboplan_traj)
    assert ros_traj.joint_names == roboplan_traj.joint_names
    assert len(ros_traj.points) == 2

    new_roboplan_traj = from_joint_trajectory(ros_traj)
    assert new_roboplan_traj.joint_names == roboplan_traj.joint_names
    assert new_roboplan_traj.times == roboplan_traj.times

    assert len(new_roboplan_traj.positions) == len(roboplan_traj.positions)
    for i in range(len(roboplan_traj.positions)):
        assert new_roboplan_traj.positions[i] == pytest.approx(
            roboplan_traj.positions[i]
        )

    assert len(new_roboplan_traj.velocities) == len(roboplan_traj.velocities)
    for i in range(len(roboplan_traj.velocities)):
        assert new_roboplan_traj.velocities[i] == pytest.approx(
            roboplan_traj.velocities[i]
        )

    assert len(new_roboplan_traj.accelerations) == len(roboplan_traj.accelerations)
    for i in range(len(roboplan_traj.accelerations)):
        assert new_roboplan_traj.accelerations[i] == pytest.approx(
            roboplan_traj.accelerations[i]
        )
