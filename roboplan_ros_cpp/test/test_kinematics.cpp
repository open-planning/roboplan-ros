#include <gtest/gtest.h>
#include <roboplan_example_models/resources.hpp>
#include <roboplan_ros_cpp/kinematics.hpp>

namespace roboplan_ros_cpp {

class RoboPlanIKTest : public ::testing::Test {
protected:
  void SetUp() override {
    const auto share_prefix = roboplan::example_models::get_install_prefix() / "share";
    const auto model_prefix = share_prefix / "roboplan_example_models" / "models";
    const auto urdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.urdf";
    const auto srdf_path = model_prefix / "ur_robot_model" / "ur5_gripper.srdf";
    const std::vector<std::filesystem::path> package_paths = {share_prefix};
    test_scene_ =
        std::make_shared<roboplan::Scene>("test_scene", urdf_path, srdf_path, package_paths);

    // Create an IK solver instance
    roboplan::SimpleIkOptions options;
    options.group_name = "arm";
    options.max_iters = 100;
    options.step_size = 0.25;
    options.damping = 0.001;
    options.max_error_norm = 0.001;
    ik_solver_ = std::make_shared<RoboPlanIK>(test_scene_, "arm", "base_link", "tool0", options);
  }

  void TearDown() override {}

  std::shared_ptr<roboplan::Scene> test_scene_;
  std::shared_ptr<RoboPlanIK> ik_solver_;
};

TEST_F(RoboPlanIKTest, SolveIK) {
  // Get a collision free position and find the FK for it
  test_scene_->setRngSeed(1234);
  Eigen::VectorXd q_full = test_scene_->randomCollisionFreePositions().value();
  test_scene_->setJointPositions(q_full);
  Eigen::Matrix4d fk_transform = test_scene_->forwardKinematics(q_full, "tool0");

  // Convert to ROS Pose and solve
  geometry_msgs::msg::Pose target_pose = se3ToPose(fk_transform);
  auto solution = ik_solver_->solveIK(target_pose);

  // Should find a solution that is reasonably close to the start pose
  ASSERT_TRUE(solution.has_value());
  Eigen::Matrix4d solution_fk = test_scene_->forwardKinematics(solution.value(), "tool0");
  Eigen::Vector3d position_error = solution_fk.block<3, 1>(0, 3) - fk_transform.block<3, 1>(0, 3);
  ASSERT_LT(position_error.norm(), 1e-5);
}

}  // namespace roboplan_ros_cpp
