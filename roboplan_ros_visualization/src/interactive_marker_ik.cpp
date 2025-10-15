#include "roboplan_ros_visualization/interactive_marker_ik.hpp"

namespace roboplan_ros_visualization {

InteractiveMarkerIK::InteractiveMarkerIK(rclcpp::Node::SharedPtr node,
                                         std::shared_ptr<roboplan::Scene> scene,
                                         std::shared_ptr<roboplan_ros_cpp::RoboPlanIK> ik_solver,
                                         IKSolvedCallback solved_callback,
                                         const std::string& namespace_name)
    : node_(node), scene_(scene), ik_solver_(ik_solver), on_ik_solved_callback_(solved_callback),
      namespace_(namespace_name) {
  // Set initial states based on the state of the scene
  last_joint_positions_ = scene_->getCurrentJointPositions();
  Eigen::Matrix4d se3_pose =
      scene_->forwardKinematics(last_joint_positions_, ik_solver_->getTipFrame());
  current_pose_ = roboplan_ros_cpp::se3ToPose(se3_pose);

  // Create the interactive marker server
  ik_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(namespace_, node_);
  createInteractiveMarker(current_pose_);
  ik_server_->applyChanges();
}

void InteractiveMarkerIK::createInteractiveMarker(const geometry_msgs::msg::Pose& pose) {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = ik_solver_->getBaseFrame();
  int_marker.name = "ik_target";
  int_marker.description = "IK Target Pose for " + namespace_;
  int_marker.pose = pose;
  int_marker.scale = 0.2;

  // Create sphere marker for visualization
  visualization_msgs::msg::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
  sphere_marker.scale.x = 0.05;
  sphere_marker.scale.y = 0.05;
  sphere_marker.scale.z = 0.05;
  sphere_marker.color.r = 0.0;
  sphere_marker.color.g = 0.5;
  sphere_marker.color.b = 1.0;
  sphere_marker.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl sphere_control;
  sphere_control.always_visible = true;
  sphere_control.markers.push_back(sphere_marker);
  int_marker.controls.push_back(sphere_control);

  // Translation controls
  visualization_msgs::msg::InteractiveMarkerControl control;

  // Move X
  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Move Y
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.orientation.z = 0.0;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Move Z
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 1.0;
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // Rotation controls
  // Rotate X
  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Rotate Y
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.orientation.z = 0.0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Rotate Z
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 1.0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  // Add the marker to the server with the feedback callback
  ik_server_->insert(int_marker, std::bind(&InteractiveMarkerIK::markerFeedbackCallback, this,
                                           std::placeholders::_1));
}

void InteractiveMarkerIK::markerFeedbackCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  // Solve IK continuously as the marker is dragged
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    current_pose_ = feedback->pose;
    solveIK(feedback->pose);
  }
}

void InteractiveMarkerIK::solveIK(const geometry_msgs::msg::Pose& target_pose) {
  auto joint_positions = ik_solver_->solveIK(target_pose);
  if (joint_positions.has_value()) {
    last_joint_positions_ = joint_positions.value();
  }
  if (on_ik_solved_callback_) {
    on_ik_solved_callback_(joint_positions, target_pose);
  }
}

void InteractiveMarkerIK::updateMarkerPose(const geometry_msgs::msg::Pose& pose) {
  ik_server_->setPose("ik_target", pose);
  ik_server_->applyChanges();
  current_pose_ = pose;
}

}  // namespace roboplan_ros_visualization
