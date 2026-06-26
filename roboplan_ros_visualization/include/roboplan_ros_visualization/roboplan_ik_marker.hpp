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

namespace roboplan_ros_visualization {

/// @brief Callback type for IK solving.
/// @param target_pose The desired end-effector pose in the base_link frame.
/// @param seed_configuration The full-model joint configuration to seed the solve from.
/// @return Full joint positions on success, std::nullopt on failure.
using IkSolveFunction = std::function<std::optional<Eigen::VectorXd>(
    const Eigen::Matrix4d& target_pose, const Eigen::VectorXd& seed_configuration)>;

/// @brief Solves IK for a target pose within a joint group with interactive marker feedback.
/// @details Can be used to generate IK solutions using feedback from an interactive
/// marker server. The consumer is responsible for connecting this to the appropriate ROS
/// infrastructure, as needed for the specific application.
class RoboplanIKMarker {
public:
  /// @brief Constructs the IK solver and marker.
  /// @param scene A fully configured RoboPlan Scene.
  /// @param joint_group The joint group name for the IK solver.
  /// @param base_link Base link of the IK chain.
  /// @param tip_link Tip link of the IK chain.
  /// @param solve_fn Callback invoked on each marker pose update to solve IK.
  RoboplanIKMarker(std::shared_ptr<const roboplan::Scene> scene, const std::string& joint_group,
                   const std::string& base_link, const std::string& tip_link,
                   IkSolveFunction ik_solve_fn);

  /// @brief Build an InteractiveMarker message for the current target pose.
  /// @return A configured InteractiveMarker with 6-DOF controls.
  visualization_msgs::msg::InteractiveMarker construct_imarker() const;

  /// @brief Process feedback from an InteractiveMarkerServer.
  /// @param feedback The feedback message from the interactive marker server.
  /// @return Joint positions if IK succeeded, std::nullopt otherwise.
  std::optional<Eigen::VectorXd>
  process_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback& feedback);

  /// @brief Sets the seed for the next IK solve (assumes the full configuration).
  void set_seed_configuration(const Eigen::VectorXd& q);

private:
  /// @brief Shared ptr to avoid ownership issues between C++ and Python.
  std::shared_ptr<const roboplan::Scene> scene_;

  /// @brief The name of the joint group in the scene
  std::string joint_group_;

  /// @brief The base link used for the IK solver, and frame for the marker header.
  std::string base_link_;

  ///@ brief The tip link of the IK solver chain.
  std::string tip_link_;

  /// @brief Function used to solve IK.
  IkSolveFunction ik_solve_fn_;

  /// @brief Current target pose for IK.
  geometry_msgs::msg::Pose target_pose_;

  /// @brief Set the seed for the IK solver.
  Eigen::VectorXd seed_configuration_;
};

}  // namespace roboplan_ros_visualization
