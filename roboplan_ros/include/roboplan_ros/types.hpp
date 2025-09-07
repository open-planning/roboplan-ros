#pragma once

#include <roboplan/core/types.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace roboplan_ros {

/// @brief Conversta a roboplan::JointTrajectory object to a ROS 2 JointTrajectory message.
/// @details This function will convert joint names and joint trajectory points. The caller
/// is responsible for any additional configuration that is required in the message.
/// @param roboplan_trajectory The roboplan JointTrajectory to convert
/// @return An equivalent ROS 2 JointTrajectory message.
inline trajectory_msgs::msg::JointTrajectory
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory) {

  trajectory_msgs::msg::JointTrajectory ros_traj;
  ros_traj.joint_names = roboplan_trajectory.joint_names;

  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < roboplan_trajectory.times.size(); ++i) {
    auto& point = ros_traj.points[i];
    double time_sec = roboplan_trajectory.times[i];
    point.time_from_start.sec = static_cast<int32_t>(time_sec);
    point.time_from_start.nanosec =
        static_cast<uint32_t>((time_sec - point.time_from_start.sec) * 1e9);

    if (i < roboplan_trajectory.positions.size()) {
      point.positions.resize(roboplan_trajectory.positions[i].size());
      for (int j = 0; j < roboplan_trajectory.positions[i].size(); ++j) {
        point.positions[j] = roboplan_trajectory.positions[i][j];
      }
    }

    if (i < roboplan_trajectory.velocities.size()) {
      point.velocities.resize(roboplan_trajectory.velocities[i].size());
      for (int j = 0; j < roboplan_trajectory.velocities[i].size(); ++j) {
        point.velocities[j] = roboplan_trajectory.velocities[i][j];
      }
    }

    if (i < roboplan_trajectory.accelerations.size()) {
      point.accelerations.resize(roboplan_trajectory.accelerations[i].size());
      for (int j = 0; j < roboplan_trajectory.accelerations[i].size(); ++j) {
        point.accelerations[j] = roboplan_trajectory.accelerations[i][j];
      }
    }
  }

  return ros_traj;
}

/// @brief Convert the provided ROS 2 JointTrajectory message to an equivalent
/// roboplan::JointTrajectory.
/// @param ros_trajectory The ROS 2 JointTrajectory message to convert.
/// @return An equivalent roboplan::JointTrajectory.
inline roboplan::JointTrajectory
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory) {

  roboplan::JointTrajectory joint_traj;

  joint_traj.joint_names = ros_trajectory.joint_names;

  joint_traj.times.reserve(ros_trajectory.points.size());
  joint_traj.positions.reserve(ros_trajectory.points.size());
  joint_traj.velocities.reserve(ros_trajectory.points.size());
  joint_traj.accelerations.reserve(ros_trajectory.points.size());

  for (const auto& point : ros_trajectory.points) {
    double time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    joint_traj.times.push_back(time_sec);

    Eigen::VectorXd positions =
        Eigen::VectorXd::Map(point.positions.data(), point.positions.size());
    joint_traj.positions.push_back(positions);

    Eigen::VectorXd velocities =
        Eigen::VectorXd::Map(point.velocities.data(), point.velocities.size());
    joint_traj.velocities.push_back(velocities);

    Eigen::VectorXd accelerations =
        Eigen::VectorXd::Map(point.accelerations.data(), point.accelerations.size());
    joint_traj.accelerations.push_back(accelerations);
  }

  return joint_traj;
}

}  // namespace roboplan_ros
