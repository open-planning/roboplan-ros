#pragma once

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

}  // namespace roboplan_ros_cpp
