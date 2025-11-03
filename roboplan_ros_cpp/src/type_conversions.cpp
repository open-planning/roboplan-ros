#include <roboplan_ros_cpp/type_conversions.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

namespace roboplan_ros_cpp {

trajectory_msgs::msg::JointTrajectory
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory) {

  trajectory_msgs::msg::JointTrajectory ros_traj;
  ros_traj.joint_names = roboplan_trajectory.joint_names;

  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < roboplan_trajectory.times.size(); ++i) {
    auto& point = ros_traj.points[i];
    double time_sec = roboplan_trajectory.times[i];
    point.time_from_start = toDuration(time_sec);

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

roboplan::JointTrajectory
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory) {

  roboplan::JointTrajectory joint_traj;

  joint_traj.joint_names = ros_trajectory.joint_names;

  joint_traj.times.reserve(ros_trajectory.points.size());
  joint_traj.positions.reserve(ros_trajectory.points.size());
  joint_traj.velocities.reserve(ros_trajectory.points.size());
  joint_traj.accelerations.reserve(ros_trajectory.points.size());

  for (const auto& point : ros_trajectory.points) {
    joint_traj.times.push_back(fromDuration(point.time_from_start));

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

geometry_msgs::msg::TransformStamped
toTransformStamped(const roboplan::CartesianConfiguration& cartesian_configuration) {
  Eigen::Isometry3d isometry(cartesian_configuration.tform);
  geometry_msgs::msg::TransformStamped ret = tf2::eigenToTransform(isometry);
  ret.header.frame_id = cartesian_configuration.base_frame;
  ret.child_frame_id = cartesian_configuration.tip_frame;
  return ret;
}

roboplan::CartesianConfiguration
fromTransformStamped(const geometry_msgs::msg::TransformStamped& transform_stamped) {
  roboplan::CartesianConfiguration config;
  config.base_frame = transform_stamped.header.frame_id;
  config.tip_frame = transform_stamped.child_frame_id;
  Eigen::Isometry3d isometry = tf2::transformToEigen(transform_stamped.transform);
  config.tform = isometry.matrix();
  return config;
}

Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d isometry;
  tf2::fromMsg(pose, isometry);
  return isometry.matrix();
}

geometry_msgs::msg::Pose se3ToPose(const Eigen::Matrix4d& transform) {
  Eigen::Isometry3d isometry(transform);
  return tf2::toMsg(isometry);
}

}  // namespace roboplan_ros_cpp
