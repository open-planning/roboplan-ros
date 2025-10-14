import pytest
import numpy as np
from pathlib import Path

from roboplan import (
    get_package_share_dir,
    Scene,
    SimpleIkOptions,
)

from roboplan_ros_py.kinematics import RoboPlanIK
from roboplan_ros_py.type_conversions import se3_to_pose


@pytest.fixture
def test_scene() -> Scene:
    """Create a test scene with UR5 robot."""
    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    return Scene("test_scene", urdf_path, srdf_path, package_paths)


@pytest.fixture
def ik_solver(test_scene: Scene) -> RoboPlanIK:
    """Create an IK solver instance."""
    options = SimpleIkOptions()
    options.group_name = "arm"
    options.max_iters = 100
    options.step_size = 0.25
    options.damping = 0.001
    options.max_error_norm = 0.001

    return RoboPlanIK(
        scene=test_scene,
        group_name="arm",
        base_frame="base_link",
        tip_frame="tool0",
        options=options,
    )


def test_ik_solver_initialization(ik_solver: RoboPlanIK) -> None:
    """Test that IK solver initializes correctly."""
    assert ik_solver.group_name == "arm"
    assert ik_solver.base_frame == "base_link"
    assert ik_solver.tip_frame == "tool0"
    assert len(ik_solver.q_indices_) == 6
    assert ik_solver.ik_solver_ is not None


def test_solve_ik(ik_solver: RoboPlanIK, test_scene: Scene) -> None:
    # Get a collision free position and find the FK for it
    test_scene.setRngSeed(1234)
    q_full = test_scene.randomCollisionFreePositions()
    test_scene.setJointPositions(q_full)
    fk_transform = test_scene.forwardKinematics(q_full, "tool0")

    # Convert to ROS Pose and solve
    target_pose = se3_to_pose(fk_transform)
    q_indices = ik_solver.q_indices_

    # Warm start from the next random position
    seed_state = test_scene.randomCollisionFreePositions()[q_indices]
    solution = ik_solver.solve_ik(target_pose, seed_state)

    # Should find a solution that is reasonably close the the start pose
    assert solution is not None
    assert len(solution) == len(q_indices)

    solution_fk = test_scene.forwardKinematics(solution, "tool0")
    assert np.linalg.norm(solution_fk[:3, 3] - fk_transform[:3, 3]) < 0.001
