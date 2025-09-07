#include <gtest/gtest.h>

#include <roboplan_ros/types.hpp>

namespace roboplan_ros {

TEST(ConversionTest, TestEmptyConversions) {
  roboplan::JointTrajectory roboplan_traj;
  const auto ros_traj = toJointTrajectory(roboplan_traj);
  EXPECT_TRUE(ros_traj.joint_names.empty());

  const auto og_roboplan_traj = fromJointTrajectory(ros_traj);
  EXPECT_TRUE(og_roboplan_traj.joint_names.empty());
}

TEST(ConversionTest, TestConvertToJointTrajectory) {
  // Shorthand helper function
  auto make_vector = [](double a, double b) {
    return (Eigen::VectorXd(2) << a, b).finished();
  };

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

} // namespace roboplan_ros
