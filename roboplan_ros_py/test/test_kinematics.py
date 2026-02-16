import pytest
import numpy as np

from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.core import Scene
from roboplan.simple_ik import SimpleIkOptions

from roboplan_ros_py.kinematics import RoboPlanIK


@pytest.fixture
def test_scene() -> Scene:
    """Create a test scene with UR5 robot."""
    roboplan_models_dir = get_package_models_dir()
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    return Scene("test_scene", urdf_path, srdf_path, package_paths)


@pytest.fixture
def ik_solver(test_scene: Scene) -> RoboPlanIK:
    """Create an IK solver instance."""
    options = SimpleIkOptions()
    options.group_name = "arm"
    options.max_iters = 100
    options.step_size = 0.25
    options.damping = 0.001
    options.max_linear_error_norm = 0.001
    options.max_angular_error_norm = 0.001
    options.check_collisions = False

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
    assert len(ik_solver._q_indices) == 6
    assert ik_solver._ik_solver is not None


def test_solve_ik(ik_solver: RoboPlanIK, test_scene: Scene) -> None:
    # Get a collision free position and find the FK for it
    test_scene.setRngSeed(1234)
    q_full = test_scene.randomCollisionFreePositions()
    test_scene.setJointPositions(q_full)
    fk_transform = test_scene.forwardKinematics(q_full, "tool0")

    # Solve and verify we get the full set of joint states pback
    solution = ik_solver.solve_ik(fk_transform)
    assert solution is not None
    assert len(q_full) == len(q_full)

    # Should find a solution that is reasonably close the the start pose
    solution_fk = test_scene.forwardKinematics(solution, "tool0")
    assert np.linalg.norm(solution_fk[:3, 3] - fk_transform[:3, 3]) < 0.001
