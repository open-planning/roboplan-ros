#include <gtest/gtest.h>

#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_cpp {

TEST(ConversionTest, TestEmptyConversions) {
  roboplan::JointTrajectory roboplan_traj;
  const auto ros_traj = toJointTrajectory(roboplan_traj);
  EXPECT_TRUE(ros_traj.joint_names.empty());

  const auto orig_roboplan_traj = fromJointTrajectory(ros_traj);
  EXPECT_TRUE(orig_roboplan_traj.joint_names.empty());
}

TEST(ConversionTest, TestConvertToJointTrajectory) {
  // Shorthand helper function
  auto make_vector = [](double a, double b) { return (Eigen::VectorXd(2) << a, b).finished(); };

  // Create a roboplan trajectory with two joints and two points
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = {"joint1", "joint2"};
  roboplan_traj.times = {0.0, 1.0};
  roboplan_traj.positions = {make_vector(0.0, 0.0), make_vector(1.0, -1.0)};
  roboplan_traj.velocities = {make_vector(0.0, 0.0), make_vector(2.0, -2.0)};
  roboplan_traj.accelerations = {make_vector(0.0, 0.0), make_vector(3.0, -3.0)};

  // Convert it and check the result, worrying about exact values below
  const auto ros_traj = toJointTrajectory(roboplan_traj);
  ASSERT_EQ(ros_traj.joint_names, roboplan_traj.joint_names);
  ASSERT_EQ(ros_traj.points.size(), 2);

  // Convert it back and we should get the original trajectory
  const auto new_roboplan_traj = fromJointTrajectory(ros_traj);
  ASSERT_EQ(new_roboplan_traj.joint_names, roboplan_traj.joint_names);
  ASSERT_EQ(new_roboplan_traj.times, roboplan_traj.times);
  ASSERT_EQ(new_roboplan_traj.positions, roboplan_traj.positions);
  ASSERT_EQ(new_roboplan_traj.velocities, roboplan_traj.velocities);
  ASSERT_EQ(new_roboplan_traj.accelerations, roboplan_traj.accelerations);
}

TEST(ConversionTest, TestPoseToSE3) {
  // Convert a random pose to SE3 then convert it back and make sure it is the same.
  geometry_msgs::msg::Pose original_pose;
  original_pose.position.x = 1.2;
  original_pose.position.y = -0.5;
  original_pose.position.z = 3.7;

  // Normalized quaternion (pi/2, -pi/2, pi/4)
  original_pose.orientation.x = 0.2705981;
  original_pose.orientation.y = -0.6532815;
  original_pose.orientation.z = -0.2705979;
  original_pose.orientation.w = 0.6532815;

  Eigen::Matrix4d transform = poseToSE3(original_pose);
  geometry_msgs::msg::Pose converted_pose = se3ToPose(transform);

  const auto tolerance = 1e-8;
  EXPECT_NEAR(original_pose.position.x, converted_pose.position.x, tolerance);
  EXPECT_NEAR(original_pose.position.y, converted_pose.position.y, tolerance);
  EXPECT_NEAR(original_pose.position.z, converted_pose.position.z, tolerance);
  EXPECT_NEAR(original_pose.orientation.w, converted_pose.orientation.w, tolerance);
  EXPECT_NEAR(original_pose.orientation.x, converted_pose.orientation.x, tolerance);
  EXPECT_NEAR(original_pose.orientation.y, converted_pose.orientation.y, tolerance);
  EXPECT_NEAR(original_pose.orientation.z, converted_pose.orientation.z, tolerance);
}

}  // namespace roboplan_ros_cpp
