#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import InteractiveMarkerFeedback

from roboplan.core import Scene, CartesianConfiguration, JointConfiguration
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.optimal_ik import (
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionLimit,
)
from roboplan_ros.cpp import se3ToPose
from roboplan_ros.visualization import RoboplanIKMarker


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

    # Build a SimpleIK solve function for use in the marker
    joint_group = "arm"
    base_link = "base_link"
    tip_link = "tool0"
    ik_solver = SimpleIk(scene, SimpleIkOptions())
    q_indices = scene.getJointGroupInfo(joint_group).q_indices

    def ik_solve_fn(target_pose, seed):
        goal = CartesianConfiguration()
        goal.base_frame = base_link
        goal.tip_frame = tip_link
        goal.tform = target_pose
        seed_jc = JointConfiguration()
        seed_jc.positions = seed[q_indices]
        solution = JointConfiguration()
        if ik_solver.solveIk(goal, seed_jc, solution):
            return scene.toFullJointPositions(joint_group, solution.positions)
        return None

    ik = RoboplanIKMarker(
        scene=scene,
        base_link=base_link,
        tip_link=tip_link,
        ik_solve_fn=ik_solve_fn,
    )

    # Compute a reachable target via FK at a known configuration
    q_target = np.array(scene.getCurrentJointPositions())
    q_target[q_indices] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    fk_target = scene.forwardKinematics(q_target, "tool0", "base_link")
    target_pose = se3ToPose(fk_target)

    # Solve via process_feedback
    feedback = InteractiveMarkerFeedback()
    feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    feedback.pose = target_pose
    result = ik.process_feedback(feedback)

    # Verify FK of solution matches the target
    assert result is not None
    fk_result = scene.forwardKinematics(result, "tool0", "base_link")
    assert abs(fk_result[0, 3] - target_pose.position.x) < 0.01
    assert abs(fk_result[1, 3] - target_pose.position.y) < 0.01
    assert abs(fk_result[2, 3] - target_pose.position.z) < 0.01


def test_process_feedback_oink():
    models_dir = get_package_models_dir()
    urdf_path = models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    scene = Scene("test", urdf_path, srdf_path, package_paths)
    q_init = np.array(scene.getCurrentJointPositions())
    q_init[:] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    scene.setJointPositions(q_init)

    joint_group = "arm"
    base_link = "base_link"
    tip_link = "tool0"

    # Configure OinK solver
    oink = Oink(scene, joint_group)
    num_variables = len(oink.v_indices)
    goal = CartesianConfiguration()
    goal.base_frame = base_link
    goal.tip_frame = tip_link
    task_options = FrameTaskOptions(
        position_cost=1.0,
        orientation_cost=0.1,
        task_gain=1.0,
        lm_damping=0.01,
    )
    frame_task = FrameTask(oink, scene, goal, task_options)
    tasks = [frame_task]

    position_limit = PositionLimit(oink, gain=1.0)
    constraints = [position_limit]
    max_iters = 50
    regularization = 1e-6

    # Setup solve function for IK marker
    def ik_solve_fn(target_pose, seed):
        scene.setJointPositions(seed)
        frame_task.setTargetFrameTransform(target_pose)

        q_current = np.array(seed)
        delta_q = np.zeros(num_variables)
        delta_q_full = np.zeros(len(seed))

        for _ in range(max_iters):
            scene.setJointPositions(q_current)
            scene.forwardKinematics(q_current, tip_link)

            try:
                oink.solveIk(scene, tasks, constraints, [], delta_q, regularization)
            except RuntimeError:
                return None

            if np.linalg.norm(delta_q) < 1e-6:
                break

            delta_q_full[oink.v_indices] = delta_q
            q_current = scene.integrate(q_current, delta_q_full)

        return q_current

    ik = RoboplanIKMarker(
        scene=scene,
        base_link=base_link,
        tip_link=tip_link,
        ik_solve_fn=ik_solve_fn,
    )

    # Compute a reachable target via FK at a known configuration
    q_target = np.array(scene.getCurrentJointPositions())
    q_indices = scene.getJointGroupInfo(joint_group).q_indices
    q_target[q_indices] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    fk_target = scene.forwardKinematics(q_target, tip_link, base_link)
    target_pose = se3ToPose(fk_target)

    # Solve via process_feedback
    feedback = InteractiveMarkerFeedback()
    feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    feedback.pose = target_pose
    result = ik.process_feedback(feedback)

    # Verify FK of solution matches the target
    assert result is not None
    fk_result = scene.forwardKinematics(result, tip_link, base_link)
    assert abs(fk_result[0, 3] - target_pose.position.x) < 0.01
    assert abs(fk_result[1, 3] - target_pose.position.y) < 0.01
    assert abs(fk_result[2, 3] - target_pose.position.z) < 0.01
