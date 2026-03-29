#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace roboplan_ros_cpp {

/// @brief Solves IK for a target pose within a joint group using an interactive marker.
/// @details Provides an out of the box IMarker for use in combination with an interactive
/// marker server. The consumer is responsible for connecting this to the appropriate ROS
/// infrastructure, as needed for the specific application.
class RoboplanIKMarker {
public:
  /// @brief Construct the IK backend.
  /// @param scene A fully configured RoboPlan Scene.
  /// @param joint_group The joint group name for the IK solver.
  /// @param base_link Base link of the IK chain.
  /// @param tip_link Tip link of the IK chain.
  /// @param options Options for the IK solver.
  RoboplanIKMarker(std::shared_ptr<const roboplan::Scene> scene, const std::string& joint_group,
                   const std::string& base_link, const std::string& tip_link,
                   const roboplan::SimpleIkOptions& options = roboplan::SimpleIkOptions());

  /// @brief Set the target pose and solve IK.
  ///
  /// Updates the internal target pose and runs the IK solver using the
  /// last successful joint positions as the seed state.
  /// @param target_pose The desired end-effector pose.
  /// @return Joint positions if a solution was found, std::nullopt otherwise.
  std::optional<Eigen::VectorXd> solve(const geometry_msgs::msg::Pose& target_pose);

  /// @brief Solve IK for the current target pose without updating it.
  /// @return Joint positions if a solution was found, std::nullopt otherwise.
  std::optional<Eigen::VectorXd> solve();

  /// @brief Set the target pose without solving.
  /// @param target_pose The desired end-effector pose.
  void set_target_pose(const geometry_msgs::msg::Pose& target_pose);

  /// @brief Get the current target pose.
  const geometry_msgs::msg::Pose& target_pose() const;

  /// @brief Get the last successful joint positions (or the initial seed).
  const Eigen::VectorXd& last_joint_positions() const;

  /// @brief Sets last joint positions, which are then used as a seed for the next solve.
  void set_joint_positions(const Eigen::VectorXd& q);

  /// @brief Build an InteractiveMarker message for the current target pose.
  /// @return A configured InteractiveMarker with 6-DOF controls.
  visualization_msgs::msg::InteractiveMarker construct_imarker() const;

  /// @brief Process feedback from an InteractiveMarkerServer.
  ///
  /// Can be called by users on iMarker pose updates.
  /// @param feedback The feedback message from the interactive marker server.
  /// @return Joint positions if IK succeeded, std::nullopt otherwise.
  std::optional<Eigen::VectorXd>
  process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback& feedback);

private:
  /// @brief Shared ptr to avoid ownership issues between C++ and Python
  std::shared_ptr<const roboplan::Scene> scene_;

  std::string joint_group_;
  std::string base_link_;
  std::string tip_link_;

  /// @brief Joint names for the active group
  std::vector<std::string> joint_names_;

  /// @brief Indices into the full configuration vector for this group
  Eigen::VectorXi q_indices_;

  /// @brief The underlying IK solver
  roboplan::SimpleIk ik_solver_;

  /// @brief Last successful joint solution, used as seed for next solve
  Eigen::VectorXd last_joint_positions_;

  /// @brief Current target pose for IK
  geometry_msgs::msg::Pose target_pose_;
};

}  // namespace roboplan_ros_cpp
