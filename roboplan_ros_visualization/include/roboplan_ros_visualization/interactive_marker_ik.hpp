#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <interactive_markers/interactive_marker_server.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_cpp/kinematics.hpp>
#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_visualization {

/// @brief Callback function signature for when IK is solved.
/// @details Parameters are: joint_positions (std::nullopt if IK failed) and target pose
using IKSolvedCallback =
    std::function<void(const std::optional<Eigen::VectorXd>&, const geometry_msgs::msg::Pose&)>;

/// @brief Utility for creating an interactive marker in RViz and passing published
/// Poses to an IK solver. Useful for nodes that ingest poses from users and
/// publish or process IK solutions for those poses.
class InteractiveMarkerIK {
public:
  /// @brief Initialize the interactive marker IK controller.
  /// @param node ROS node instance to be passed to the InteractiveMarker server.
  /// @param scene A fully configured RoboPlan scene.
  /// @param ik_solver RoboPlanIK instance for solving IK.
  /// @param solved_callback Function to be called each time an IK solution is found.
  ///                        Requires signature `callback(joint_positions, pose)`,
  ///                        where joint_positions is std::optional<Eigen::VectorXd>.
  /// @param namespace_name Namespace for the InteractiveMarkerServer.
  InteractiveMarkerIK(rclcpp::Node::SharedPtr node, std::shared_ptr<roboplan::Scene> scene,
                      std::shared_ptr<roboplan_ros_cpp::RoboPlanIK> ik_solver,
                      IKSolvedCallback solved_callback = nullptr,
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
  /// @param target_pose The target pose for the IK solver.
  void solveIK(const geometry_msgs::msg::Pose& target_pose);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<roboplan::Scene> scene_;
  std::shared_ptr<roboplan_ros_cpp::RoboPlanIK> ik_solver_;
  IKSolvedCallback on_ik_solved_callback_;
  std::string namespace_;

  Eigen::VectorXd last_joint_positions_;
  geometry_msgs::msg::Pose current_pose_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> ik_server_;
};

}  // namespace roboplan_ros_visualization
