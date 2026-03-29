#include <roboplan_ros_cpp/type_conversions.hpp>
#include <roboplan_ros_visualization/roboplan_ik_marker.hpp>

namespace roboplan_ros_cpp {

RoboplanIKMarker::RoboplanIKMarker(std::shared_ptr<const roboplan::Scene> scene,
                                   const std::string& joint_group, const std::string& base_link,
                                   const std::string& tip_link,
                                   const roboplan::SimpleIkOptions& options)
    : scene_(std::move(scene)), joint_group_(joint_group), base_link_(base_link),
      tip_link_(tip_link) {
  ik_solver_ = std::make_unique<roboplan::SimpleIk>(
      std::const_pointer_cast<roboplan::Scene>(scene_), options);

  const auto joint_group_result = scene_->getJointGroupInfo(joint_group_);
  if (!joint_group_result.has_value()) {
    throw std::runtime_error("Failed to get joint group info: " + joint_group_result.error());
  }
  q_indices_ = joint_group_result->q_indices;

  const auto& all_joint_names = scene_->getJointNames();
  joint_names_.clear();
  joint_names_.reserve(q_indices_.size());
  for (Eigen::Index i = 0; i < q_indices_.size(); ++i) {
    joint_names_.push_back(all_joint_names.at(q_indices_[i]));
  }

  last_joint_positions_ = scene_->getCurrentJointPositions();
  const auto se3_pose = scene_->forwardKinematics(last_joint_positions_, tip_link_);
  target_pose_ = roboplan_ros_cpp::se3ToPose(se3_pose);
}

visualization_msgs::msg::InteractiveMarker RoboplanIKMarker::construct_imarker() const {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = base_link_;
  int_marker.name = "ik_target";
  int_marker.description = "IK Target Pose for " + joint_group_;
  int_marker.pose = target_pose_;
  int_marker.scale = 0.2f;

  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.scale.x = 0.025;
  sphere.scale.y = 0.025;
  sphere.scale.z = 0.025;
  sphere.color.r = 0.0f;
  sphere.color.g = 0.5f;
  sphere.color.b = 1.0f;
  sphere.color.a = 1.0f;

  visualization_msgs::msg::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.markers.push_back(sphere);
  int_marker.controls.push_back(sphere_control);

  struct AxisDef {
    double x, y, z;
    const char* move_name;
    const char* rotate_name;
  };

  const AxisDef axes[] = {
      {1.0, 0.0, 0.0, "move_x", "rotate_x"},
      {0.0, 1.0, 0.0, "move_y", "rotate_y"},
      {0.0, 0.0, 1.0, "move_z", "rotate_z"},
  };

  for (const auto& axis : axes) {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = axis.x;
    control.orientation.y = axis.y;
    control.orientation.z = axis.z;

    control.name = axis.move_name;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = axis.rotate_name;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }

  return int_marker;
}

std::optional<Eigen::VectorXd> RoboplanIKMarker::process_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback& feedback) {
  if (feedback.event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    return std::nullopt;
  }

  target_pose_ = feedback.pose;
  const Eigen::Matrix4d tform = roboplan_ros_cpp::poseToSE3(target_pose_);

  roboplan::CartesianConfiguration goal;
  goal.base_frame = base_link_;
  goal.tip_frame = tip_link_;
  goal.tform = tform;

  // Extract only the group joints for the seed
  roboplan::JointConfiguration seed;
  Eigen::VectorXd group_positions(q_indices_.size());
  for (Eigen::Index i = 0; i < q_indices_.size(); ++i) {
    group_positions[i] = last_joint_positions_[q_indices_[i]];
  }
  seed.positions = group_positions;

  roboplan::JointConfiguration solution;
  const bool success = ik_solver_->solveIk(goal, seed, solution);

  if (success) {
    // Then we need to expand back to the full state
    for (Eigen::Index i = 0; i < q_indices_.size(); ++i) {
      last_joint_positions_[q_indices_[i]] = solution.positions[i];
    }
    return last_joint_positions_;
  }

  return std::nullopt;
}

const Eigen::VectorXd& RoboplanIKMarker::last_joint_positions() const {
  return last_joint_positions_;
}

void RoboplanIKMarker::set_joint_positions(const Eigen::VectorXd& q) { last_joint_positions_ = q; }

}  // namespace roboplan_ros_cpp
