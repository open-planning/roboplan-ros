#include "roboplan_ros_visualization/interactive_marker_ik.hpp"

namespace roboplan_ros_visualization {

InteractiveMarkerIK::InteractiveMarkerIK(rclcpp::Node::SharedPtr node,
                                         std::shared_ptr<roboplan::Scene> scene,
                                         const std::string& joint_group,
                                         const std::string& base_link, const std::string& tip_link,
                                         const roboplan::SimpleIkOptions& options,
                                         const std::string& namespace_name)
    : node_(node), scene_(scene), namespace_(namespace_name), joint_group_(joint_group) {

  // Get joint group information
  auto joint_group_info = scene_->getJointGroupInfo(joint_group_);
  if (!joint_group_info.has_value()) {
    throw std::runtime_error("Failed to get joint group info for: " + joint_group_);
  }

  // Construct the IK solver
  ik_solver_ = std::make_shared<roboplan_ros_cpp::RoboPlanIK>(scene_, joint_group_, base_link,
                                                              tip_link, options);

  // Set initial states based on the state of the scene
  last_joint_positions_ = scene_->getCurrentJointPositions();
  Eigen::Matrix4d se3_pose = scene_->forwardKinematics(last_joint_positions_, tip_link);
  current_pose_ = roboplan_ros_cpp::se3ToPose(se3_pose);

  // Create joint state publisher
  std::string joint_state_topic = namespace_name + "/joint_states";
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 1);

  // Create the interactive marker server
  ik_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(namespace_, node_);
  createInteractiveMarker(current_pose_);
  ik_server_->applyChanges();

  // Solve and publish joint states at 20 Hz
  joint_state_timer_ =
      node_->create_wall_timer(std::chrono::milliseconds(50), [this]() { solveIK(); });

  RCLCPP_INFO(node_->get_logger(), "Constructed interactive marker controller");
  RCLCPP_INFO(node_->get_logger(), "Joint group: %s", joint_group_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Base link: %s", base_link.c_str());
  RCLCPP_INFO(node_->get_logger(), "End-effector: %s", tip_link.c_str());
}

void InteractiveMarkerIK::createInteractiveMarker(const geometry_msgs::msg::Pose& pose) {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = namespace_ + "/" + ik_solver_->getBaseFrame();
  int_marker.name = "ik_target";
  int_marker.description = "IK Target Pose for " + joint_group_;
  int_marker.pose = pose;
  int_marker.scale = 0.2;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.orientation.z = 0.0;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 0.0;
  control.orientation.z = 1.0;
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1.0;
  control.orientation.x = 1.0;
  control.orientation.y = 0.0;
  control.orientation.z = 0.0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.orientation.z = 0.0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

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
  // On pose updates set the latest pose and let the timer manage solving for joint states
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_pose_ = feedback->pose;
  }
}

void InteractiveMarkerIK::solveIK() {
  auto joint_positions = ik_solver_->solveIK(current_pose_, last_joint_positions_);

  if (joint_positions.has_value()) {
    last_joint_positions_ = joint_positions.value();
    publishJointStates();
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "IK failed to find solution for target pose");
  }
}

void InteractiveMarkerIK::publishJointStates() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->now();
  msg.header.frame_id = "";
  msg.name = scene_->getJointNames();

  {
    // Convert Eigen to std::vector
    std::lock_guard<std::mutex> lock(state_mutex_);
    msg.position.resize(last_joint_positions_.size());
    for (int i = 0; i < last_joint_positions_.size(); ++i) {
      msg.position[i] = last_joint_positions_(i);
    }
  }

  joint_state_pub_->publish(msg);
}

void InteractiveMarkerIK::updateMarkerPose(const geometry_msgs::msg::Pose& pose) {
  ik_server_->setPose("ik_target", pose);
  ik_server_->applyChanges();
  current_pose_ = pose;
}

}  // namespace roboplan_ros_visualization
