import pytest
import numpy as np

from ament_index_python.packages import get_package_share_path
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped

from roboplan.core import (
    Scene,
    JointConfiguration,
    JointTrajectory,
    loadTextFile,
    loadUrdfSceneDescription,
)
from roboplan_ros.cpp import (
    buildConversionMap,
    toJointState,
    fromJointState,
    toJointTrajectory,
    toMultiDOFJointTrajectory,
    fromJointTrajectory,
    se3ToPose,
    poseToSE3,
    toTransformStamped,
    fromTransformStamped,
)


resource_path = get_package_share_path("roboplan_ros_cpp") / "test" / "resources"
urdf_path = resource_path / "test_robot.urdf"
srdf_path = resource_path / "test_robot.srdf"
mobile_urdf_path = resource_path / "test_mobile_robot.urdf"


def test_joint_state_mapping():
    scene = Scene("test_scene", loadUrdfSceneDescription(urdf_path))
    scene.importSrdf(loadTextFile(srdf_path))
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
    scene = Scene("test_scene", loadUrdfSceneDescription(urdf_path))
    scene.importSrdf(loadTextFile(srdf_path))
    scene.setRngSeed(1234)

    joint_state = JointState()
    joint_state.name = scene.getJointNames()
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


def test_convert_partial_joint_state():
    """Test that joints missing from the JointState keep their reference positions."""
    scene = Scene("test_scene", loadUrdfSceneDescription(urdf_path))
    scene.importSrdf(loadTextFile(srdf_path))
    scene.setRngSeed(1234)
    q_current = scene.randomPositions()
    scene.setJointPositions(q_current)

    # The message has the revolute joint only, not the continuous one.
    joint_state = JointState()
    joint_state.name = ["revolute_joint"]
    joint_state.position = [0.3]
    conversion_map = buildConversionMap(scene, joint_state)
    continuous_idx = scene.getJointPositionIndices(["continuous_joint"])
    revolute_idx = scene.getJointPositionIndices(["revolute_joint"])

    # Without a reference, the missing joint keeps the scene's current position.
    from_current = fromJointState(joint_state, scene, conversion_map)
    assert np.allclose(
        from_current.positions[continuous_idx], q_current[continuous_idx]
    )
    assert from_current.positions[revolute_idx[0]] == pytest.approx(0.3)

    # With a reference, it keeps the reference's position instead.
    q_reference = scene.randomPositions()
    from_reference = fromJointState(joint_state, scene, conversion_map, q_reference)
    assert np.allclose(
        from_reference.positions[continuous_idx], q_reference[continuous_idx]
    )
    assert from_reference.positions[revolute_idx[0]] == pytest.approx(0.3)

    # A reference of the wrong size is an error.
    with pytest.raises(RuntimeError) as exc_info:
        fromJointState(joint_state, scene, conversion_map, np.zeros(1))
    expected_error = f"q_reference has 1 positions, the scene has {len(q_current)}"
    assert str(exc_info.value) == expected_error


def test_empty_conversions():
    scene = Scene("test_scene", loadUrdfSceneDescription(urdf_path))
    roboplan_traj = JointTrajectory()
    ros_traj = toJointTrajectory(roboplan_traj, scene)
    assert len(ros_traj.joint_names) == 0

    orig_roboplan_traj = fromJointTrajectory(ros_traj, scene)
    assert len(orig_roboplan_traj.joint_names) == 0


def test_convert_to_joint_trajectory():
    # Create a roboplan trajectory with a revolute joint and two points, convert it
    # to/from ROS and ensure we get the same data back.
    scene = Scene("test_scene", loadUrdfSceneDescription(urdf_path))
    roboplan_traj = JointTrajectory()
    roboplan_traj.joint_names = ["revolute_joint"]
    roboplan_traj.times = [0.0, 1.0]
    roboplan_traj.positions = [np.array([0.0]), np.array([1.0])]
    roboplan_traj.velocities = [np.array([0.0]), np.array([2.0])]
    roboplan_traj.accelerations = [np.array([0.0]), np.array([3.0])]

    ros_traj = toJointTrajectory(roboplan_traj, scene)
    assert ros_traj.joint_names == roboplan_traj.joint_names
    assert len(ros_traj.points) == 2

    new_roboplan_traj = fromJointTrajectory(ros_traj, scene)
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


def test_convert_joint_trajectory_with_continuous_joint():
    """Test that a scene converts the positions of continuous joints."""
    scene = Scene("test_scene", loadUrdfSceneDescription(urdf_path))

    # RoboPlan stores the continuous joint as [cos(theta), sin(theta)], ROS as theta.
    roboplan_traj = JointTrajectory()
    roboplan_traj.joint_names = ["continuous_joint", "revolute_joint"]
    roboplan_traj.times = [0.0, 1.0]
    roboplan_traj.positions = [
        np.array([np.cos(0.5), np.sin(0.5), 0.1]),
        np.array([np.cos(-2.0), np.sin(-2.0), 0.2]),
    ]
    roboplan_traj.velocities = [np.array([0.0, 0.0]), np.array([1.0, -1.0])]
    roboplan_traj.accelerations = [np.array([0.0, 0.0]), np.array([2.0, -2.0])]

    ros_traj = toJointTrajectory(roboplan_traj, scene)
    assert ros_traj.joint_names == roboplan_traj.joint_names
    assert list(ros_traj.points[1].positions) == pytest.approx([-2.0, 0.2])

    new_roboplan_traj = fromJointTrajectory(ros_traj, scene)
    for i in range(len(roboplan_traj.positions)):
        assert new_roboplan_traj.positions[i] == pytest.approx(
            roboplan_traj.positions[i]
        )

    # A wrong number of positions is an error.
    ros_traj.points[0].positions = [0.5]
    with pytest.raises(RuntimeError) as exc_info:
        fromJointTrajectory(ros_traj, scene)
    expected_error = (
        "Failed to convert the trajectory: Point 0 has 1 positions, expected 2"
    )
    assert str(exc_info.value) == expected_error


def test_convert_multi_dof_joint_trajectory():
    """Test that planar and floating joints go in a MultiDOFJointTrajectory."""
    scene = Scene("test_scene", loadUrdfSceneDescription(mobile_urdf_path))

    # Planar [x, y, cos, sin], continuous [cos, sin],
    # floating [x, y, z, qx, qy, qz, qw].
    def make_q(base_theta):
        return np.array(
            [1.0, 2.0, np.cos(base_theta), np.sin(base_theta)]
            + [np.cos(0.5), np.sin(0.5)]
            + [0.1, 0.2, 1.0, 0.0, 0.0, 0.0, 1.0]
        )

    velocities = np.array([0.1, 0.2, 0.3, 0.4, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    roboplan_traj = JointTrajectory()
    roboplan_traj.joint_names = ["base_joint", "wheel_joint", "drone_joint"]
    roboplan_traj.times = [0.0, 1.0]
    roboplan_traj.positions = [make_q(3.0), make_q(-3.0)]
    roboplan_traj.velocities = [velocities, velocities]

    ros_traj = toJointTrajectory(roboplan_traj, scene)
    assert ros_traj.joint_names == ["wheel_joint"]

    multi_dof_traj = toMultiDOFJointTrajectory(roboplan_traj, scene)
    assert multi_dof_traj.joint_names == ["base_joint", "drone_joint"]
    base_twist = multi_dof_traj.points[0].velocities[0]
    assert [base_twist.linear.x, base_twist.linear.y, base_twist.angular.z] == [
        0.1,
        0.2,
        0.3,
    ]

    # The joints of the JointTrajectory come first: the wheel, then base and drone.
    new_roboplan_traj = fromJointTrajectory(ros_traj, multi_dof_traj, scene)
    assert new_roboplan_traj.joint_names == ["wheel_joint", "base_joint", "drone_joint"]
    for i in range(len(roboplan_traj.positions)):
        q, v = roboplan_traj.positions[i], roboplan_traj.velocities[i]
        assert new_roboplan_traj.positions[i] == pytest.approx(
            np.concatenate([q[4:6], q[0:4], q[6:]])
        )
        assert new_roboplan_traj.velocities[i] == pytest.approx(
            np.concatenate([v[3:4], v[0:3], v[4:]])
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
