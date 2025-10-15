#include "roboplan_ros_cpp/kinematics.hpp"

namespace roboplan_ros_cpp {

RoboPlanIK::RoboPlanIK(std::shared_ptr<roboplan::Scene> scene, const std::string& group_name,
                       const std::string& base_frame, const std::string& tip_frame,
                       const roboplan::SimpleIkOptions& options)
    : scene_(scene), group_name_(group_name), base_frame_(base_frame), tip_frame_(tip_frame),
      options_(options) {

  // Initialize the IK solver
  ik_solver_ = std::make_shared<roboplan::SimpleIk>(scene_, options_);

  // Get joint group information
  const auto maybe_group_info = scene_->getJointGroupInfo(group_name_);
  if (!maybe_group_info.has_value()) {
    throw std::runtime_error("Failed to get joint group: " + maybe_group_info.error());
  }
  joint_group_info_ = maybe_group_info.value();
  q_indices_ = joint_group_info_.q_indices;
}

std::optional<Eigen::VectorXd>
RoboPlanIK::solveIK(const geometry_msgs::msg::Pose& target_pose,
                    const std::optional<Eigen::VectorXd>& seed_state) {

  // Convert pose to a transform
  Eigen::Matrix4d target_transform = poseToSE3(target_pose);

  // Determine a reasonable start pose using the seed provided, or else
  // use the current pose.
  roboplan::JointConfiguration start;
  if (seed_state.has_value()) {
    start.positions = seed_state.value();
  } else {
    start.positions = scene_->getCurrentJointPositions()(q_indices_);
  }

  // Configure the goal pose and solve.
  roboplan::CartesianConfiguration goal;
  goal.base_frame = base_frame_;
  goal.tip_frame = tip_frame_;
  goal.tform = target_transform;

  roboplan::JointConfiguration solution;
  bool result = ik_solver_->solveIk(goal, start, solution);

  // Return the solved position if present, or nothing.
  if (result) {
    return solution.positions;
  } else {
    return std::nullopt;
  }
}

}  // namespace roboplan_ros_cpp
