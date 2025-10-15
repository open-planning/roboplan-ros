#include <roboplan_ros_cpp/type_conversions.hpp>

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

Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& pose) {
  Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  pinocchio::SE3::Quaternion quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z);

  pinocchio::SE3 se3(quaternion, position);
  return se3.toHomogeneousMatrix();
}

geometry_msgs::msg::Pose se3ToPose(const Eigen::Matrix4d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform(0, 3);
  pose.position.y = transform(1, 3);
  pose.position.z = transform(2, 3);
  Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  pinocchio::SE3::Quaternion quat(rotation);
  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  return pose;
}

}  // namespace roboplan_ros_cpp
