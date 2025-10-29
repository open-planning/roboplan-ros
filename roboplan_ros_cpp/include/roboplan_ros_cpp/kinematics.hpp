#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_cpp {

/// @brief IK solver wrapper for roboplan Scene objects with ROS message support.
/// This class provides methods to solve IK given ROS Pose messages and
/// returns results as Eigen vectors.
class RoboPlanIK {

public:
  /// @brief Construct an IK solver.
  /// @param scene RoboPlan Scene object.
  /// @param group_name Name of the joint group to use.
  /// @param base_frame Base frame for IK.
  /// @param tip_frame End effector/tip frame for IK.
  /// @param options Options for the IK solver.
  RoboPlanIK(std::shared_ptr<roboplan::Scene> scene, const std::string& group_name,
             const std::string& base_frame, const std::string& tip_frame,
             const roboplan::SimpleIkOptions& options);

  /// @brief Solve IK for a target pose.
  /// @param target_pose ROS Pose message with target position and orientation.
  /// @param seed_state Optional seed joint positions, defaults to current pose. Users MUST pass
  ///                   a full set of joint positions.
  /// @return A set of joint positions for the desired pose, or std::nullopt if IK failed.
  std::optional<Eigen::VectorXd>
  solveIK(const geometry_msgs::msg::Pose& target_pose,
          const std::optional<Eigen::VectorXd>& seed_state = std::nullopt);

  /// @brief Get the base frame name
  /// @return Base frame name
  std::string getBaseFrame() const { return base_frame_; }

  /// @brief Get the tip frame name
  /// @return Tip frame name
  std::string getTipFrame() const { return tip_frame_; }

  /// @brief Get the joint group name
  /// @return Joint group name
  std::string getGroupName() const { return group_name_; }

private:
  std::shared_ptr<roboplan::Scene> scene_;
  std::string group_name_;
  std::string base_frame_;
  std::string tip_frame_;
  roboplan::SimpleIkOptions options_;

  std::shared_ptr<roboplan::SimpleIk> ik_solver_;
  roboplan::JointGroupInfo joint_group_info_;
  Eigen::VectorXi q_indices_;
};

}  // namespace roboplan_ros_cpp
