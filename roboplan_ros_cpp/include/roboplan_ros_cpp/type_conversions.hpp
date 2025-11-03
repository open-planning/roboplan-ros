#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <roboplan/core/types.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace roboplan_ros_cpp {

/// @brief Converts a double timestamp to an equivalent ROS Duration.
/// @param time Timestamp in seconds.
/// @return ROS 2 Duration with seconds and nanoseconds.
inline builtin_interfaces::msg::Duration toDuration(const double time_sec) {
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(time_sec);
  duration.nanosec = static_cast<uint32_t>((time_sec - duration.sec) * 1e9);
  return duration;
}

/// @brief Converts a ROS 2 Duration with seconds and nanoseconds to an equivalent double timestamp.
/// @param duration ROS 2 Duration with seconds and nanoseconds.
/// @return An equivalent double timestamp.
inline double fromDuration(const builtin_interfaces::msg::Duration& duration) {
  return duration.sec + duration.nanosec * 1e-9;
}

/// @brief Converts a roboplan::JointTrajectory object to a ROS 2 JointTrajectory message.
/// @details This function will convert joint names and joint trajectory points. The caller
/// is responsible for any additional configuration that is required in the message.
/// @param roboplan_trajectory The roboplan JointTrajectory to convert
/// @return An equivalent ROS 2 JointTrajectory message.
trajectory_msgs::msg::JointTrajectory
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory);

/// @brief Convert the provided ROS 2 JointTrajectory message to an equivalent
/// roboplan::JointTrajectory.
/// @param ros_trajectory The ROS 2 JointTrajectory message to convert.
/// @return An equivalent roboplan::JointTrajectory.
roboplan::JointTrajectory
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory);

/// @brief Converts a roboplan::CartesianConfiguration to a ROS 2 TransformStamped message.
/// @param cartesian_configuration The roboplan CartesianConfiguration to convert.
/// @return An equivalent geometry_msgs::msg::TransformStamped.
geometry_msgs::msg::TransformStamped
toTransformStamped(const roboplan::CartesianConfiguration& cartesian_configuration);

/// @brief Converts a geometry_msgs::msg::TransformStamped to a roboplan CartesianConfiguration.
/// @param transform The ROS 2 TransformStamped message to convert.
/// @return An equivalent roboplan::CartesianConfiguration.
roboplan::CartesianConfiguration
fromTransformStamped(const geometry_msgs::msg::TransformStamped& transform);

/// @brief Convert ROS Pose to a 4x4 SE3 transformation matrix.
/// @param pose ROS Pose message
/// @return 4x4 Eigen matrix representing the SE3 transformation
Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& pose);

/// @brief Convert a 4x4 SE3 transformation matrix to ROS Pose.
/// @param transform 4x4 Eigen matrix representing an SE3 transformation
/// @return An equivalent ROS Pose message
geometry_msgs::msg::Pose se3ToPose(const Eigen::Matrix4d& transform);

}  // namespace roboplan_ros_cpp
