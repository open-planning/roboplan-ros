#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import InteractiveMarkerFeedback

from roboplan.core import Scene
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan_ros_cpp.bindings import se3ToPose
from roboplan_ros_visualization.bindings import RoboplanIKMarker


def test_process_feedback():
    models_dir = get_package_models_dir()
    urdf_path = models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    # Just start right next to IK pose to make the IK trivial
    scene = Scene("test", urdf_path, srdf_path, package_paths)
    q_init = np.array(scene.getCurrentJointPositions())
    q_init[:] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    scene.setJointPositions(q_init)

    ik = RoboplanIKMarker(
        scene=scene,
        joint_group="arm",
        base_link="base_link",
        tip_link="tool0",
    )

    # Compute a reachable target via FK at a known configuration
    q_target = np.array(scene.getCurrentJointPositions())
    q_indices = scene.getJointGroupInfo("arm").q_indices
    q_target[q_indices] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    fk_target = scene.forwardKinematics(q_target, "tool0")
    target_pose = se3ToPose(fk_target)

    # Solve via process_feedback
    feedback = InteractiveMarkerFeedback()
    feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    feedback.pose = target_pose
    result = ik.process_feedback(feedback)

    # Verify FK of solution matches the target
    assert result is not None
    fk_result = scene.forwardKinematics(result, "tool0")
    assert abs(fk_result[0, 3] - target_pose.position.x) < 0.01
    assert abs(fk_result[1, 3] - target_pose.position.y) < 0.01
    assert abs(fk_result[2, 3] - target_pose.position.z) < 0.01
