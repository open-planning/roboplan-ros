#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_cpp/kinematics.hpp>
#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_visualization {

/// @brief Utility for creating an interactive marker in RViz and passing published
/// Poses to an IK solver.
class InteractiveMarkerIK {
public:
  /// @brief Initialize the interactive marker IK controller.
  /// @param node ROS node instance to be passed to the InteractiveMarker server.
  /// @param scene A fully configured RoboPlan scene.
  /// @param joint_group Name of the joint group for IK.
  /// @param base_link Base link for the IK chain.
  /// @param tip_link Tip link for the IK chain.
  /// @param options IK solver options.
  /// @param namespace_name Namespace for the InteractiveMarkerServer and joint state publisher.
  InteractiveMarkerIK(rclcpp::Node::SharedPtr node, std::shared_ptr<roboplan::Scene> scene,
                      const std::string& joint_group, const std::string& base_link,
                      const std::string& tip_link,
                      const roboplan::SimpleIkOptions& options = roboplan::SimpleIkOptions(),
                      const std::string& namespace_name = "roboplan_ik");

  /// @brief Get the current pose of the interactive marker.
  /// @return Current pose
  geometry_msgs::msg::Pose getCurrentPose() const { return current_pose_; }

  /// @brief Get the last computed joint positions.
  /// @return Last joint positions
  Eigen::VectorXd getLastJointPositions() const { return last_joint_positions_; }

  /// @brief Update the marker pose.
  /// @param pose New pose for the marker
  void updateMarkerPose(const geometry_msgs::msg::Pose& pose);

private:
  /// @brief Constructs a new interactive marker at the specified pose.
  /// @param pose Initial pose for the marker
  void createInteractiveMarker(const geometry_msgs::msg::Pose& pose);

  /// @brief Callback function to process pose updates from the interactive marker.
  /// @param feedback InteractiveMarkerFeedback message
  void markerFeedbackCallback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  /// @brief Solve IK for the target pose.
  void solveIK();

  /// @brief Publish current joint states.
  void publishJointStates();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<roboplan::Scene> scene_;
  std::shared_ptr<roboplan_ros_cpp::RoboPlanIK> ik_solver_;
  std::string namespace_;
  std::string joint_group_;

  // Protects the state variables below
  mutable std::mutex state_mutex_;
  Eigen::VectorXd last_joint_positions_;
  geometry_msgs::msg::Pose current_pose_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> ik_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
};

}  // namespace roboplan_ros_visualization
