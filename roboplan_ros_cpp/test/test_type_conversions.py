import pytest
import numpy as np
import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped

from roboplan.core import Scene, JointConfiguration, JointTrajectory
from roboplan_ros_cpp.bindings import (
    buildConversionMap,
    toJointState,
    fromJointState,
    toJointTrajectory,
    fromJointTrajectory,
    se3ToPose,
    poseToSE3,
    toTransformStamped,
    fromTransformStamped,
)


resource_path = (
    Path(get_package_share_directory("roboplan_ros_cpp")) / "test" / "resources"
)
urdf_path = resource_path / "test_robot.urdf"
srdf_path = resource_path / "test_robot.srdf"


def test_joint_state_mapping():
    scene = Scene("test_scene", urdf_path, srdf_path)
    joint_state = JointState()
    joint_state.name = ["continuous_joint", "revolute_joint"]
    conversion_map = buildConversionMap(scene, joint_state)

    # Verify what we can since we don't have direct access to the Pinocchio model
    assert len(conversion_map.mappings) == 2
    for i, mapping in enumerate(conversion_map.mappings):
        assert mapping.joint_name == joint_state.name[i]
        assert mapping.ros_index == i


def test_convert_joint_state():
    """Test converting between JointConfiguration and JointState."""
    scene = Scene("test_scene", urdf_path, srdf_path)
    scene.setRngSeed(1234)

    joint_state = JointState()
    joint_state.name = scene.getActuatedJointNames()
    conversion_map = buildConversionMap(scene, joint_state)

    # Setup a joint configuration
    joint_configuration = JointConfiguration()
    joint_configuration.joint_names = scene.getJointNames()
    joint_configuration.positions = scene.randomPositions()
    joint_configuration.velocities = np.zeros(conversion_map.nv)
    joint_configuration.accelerations = np.zeros(conversion_map.nv)

    # Convert to ROS JointState and back
    ros_joint_state = toJointState(joint_configuration, scene)
    check_joint_configuration = fromJointState(ros_joint_state, scene, conversion_map)

    # Verify we get the same values back
    assert len(ros_joint_state.name) == 2
    assert joint_configuration.joint_names == check_joint_configuration.joint_names
    assert np.allclose(
        joint_configuration.positions, check_joint_configuration.positions
    )
    assert np.allclose(
        joint_configuration.velocities, check_joint_configuration.velocities
    )
    assert np.allclose(
        joint_configuration.accelerations, check_joint_configuration.accelerations
    )


def test_empty_conversions():
    roboplan_traj = JointTrajectory()
    ros_traj = toJointTrajectory(roboplan_traj)
    assert len(ros_traj.joint_names) == 0

    orig_roboplan_traj = fromJointTrajectory(ros_traj)
    assert len(orig_roboplan_traj.joint_names) == 0


def test_convert_to_joint_trajectory():
    # Create a roboplan trajectory with two joints and two points, convert it to/from
    # ROS and ensure we get the same data back.
    roboplan_traj = JointTrajectory()
    roboplan_traj.joint_names = ["joint1", "joint2"]
    roboplan_traj.times = [0.0, 1.0]
    roboplan_traj.positions = [np.array([0.0, 0.0]), np.array([1.0, -1.0])]
    roboplan_traj.velocities = [np.array([0.0, 0.0]), np.array([2.0, -2.0])]
    roboplan_traj.accelerations = [np.array([0.0, 0.0]), np.array([3.0, -3.0])]

    ros_traj = toJointTrajectory(roboplan_traj)
    assert ros_traj.joint_names == roboplan_traj.joint_names
    assert len(ros_traj.points) == 2

    new_roboplan_traj = fromJointTrajectory(ros_traj)
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


def test_transform_stamped_conversion():
    # Given a transform convert it to/from a CartesianConfiguration and validate it is the same.
    original_transform = TransformStamped()
    original_transform.header.frame_id = "world"
    original_transform.child_frame_id = "end_effector"

    original_transform.transform.translation.x = 1.2
    original_transform.transform.translation.y = -0.5
    original_transform.transform.translation.z = 3.7
    original_transform.transform.rotation.w = 0.6532815
    original_transform.transform.rotation.x = 0.2705981
    original_transform.transform.rotation.y = -0.6532815
    original_transform.transform.rotation.z = -0.2705979

    cartesian_config = fromTransformStamped(original_transform)
    converted_transform = toTransformStamped(cartesian_config)

    assert original_transform.header.frame_id == converted_transform.header.frame_id
    assert original_transform.child_frame_id == converted_transform.child_frame_id

    assert converted_transform.transform.translation.x == pytest.approx(
        original_transform.transform.translation.x
    )
    assert converted_transform.transform.translation.y == pytest.approx(
        original_transform.transform.translation.y
    )
    assert converted_transform.transform.translation.z == pytest.approx(
        original_transform.transform.translation.z
    )
    assert converted_transform.transform.rotation.w == pytest.approx(
        original_transform.transform.rotation.w
    )
    assert converted_transform.transform.rotation.x == pytest.approx(
        original_transform.transform.rotation.x
    )
    assert converted_transform.transform.rotation.y == pytest.approx(
        original_transform.transform.rotation.y
    )
    assert converted_transform.transform.rotation.z == pytest.approx(
        original_transform.transform.rotation.z
    )


def test_pose_to_se3():
    # Given a pose convert it to/from an SE3 matrix and validate it is the same.
    original_pose = Pose()
    original_pose.position.x = 1.2
    original_pose.position.y = -0.5
    original_pose.position.z = 3.7
    original_pose.orientation.x = 0.2705981
    original_pose.orientation.y = -0.6532815
    original_pose.orientation.z = -0.2705979
    original_pose.orientation.w = 0.6532815

    transform = poseToSE3(original_pose)
    converted_pose = se3ToPose(transform)

    assert converted_pose.position.x == pytest.approx(original_pose.position.x)
    assert converted_pose.position.y == pytest.approx(original_pose.position.y)
    assert converted_pose.position.z == pytest.approx(original_pose.position.z)

    assert converted_pose.orientation.w == pytest.approx(original_pose.orientation.w)
    assert converted_pose.orientation.x == pytest.approx(original_pose.orientation.x)
    assert converted_pose.orientation.y == pytest.approx(original_pose.orientation.y)
    assert converted_pose.orientation.z == pytest.approx(original_pose.orientation.z)
