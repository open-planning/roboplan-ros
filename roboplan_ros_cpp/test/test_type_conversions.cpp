#include <gtest/gtest.h>

#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_cpp {

// Sample URDF and SRDF for testing ROS to RoboPlan joint mappings
// for different types of joints.
const std::string URDF = R"(
<robot name="robot">
  <link name="base_link"/>
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <joint name="continuous_joint" type="continuous">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="revolute_joint" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <joint name="mimic_joint" type="revolute">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
    <mimic joint="revolute_joint" multiplier="1.0" offset="0.0"/>
  </joint>
</robot>
)";

const std::string SRDF = R"(
<robot name="robot">
  <group name="arm">
    <joint name="revolute_joint"/>
    <joint name="mimic_joint"/>
  </group>
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
  <disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
  <disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
</robot>
)";

TEST(TypeConversions, TestJointStateMapping) {
  // Setup the Scene and ROS JointState message
  const auto scene = roboplan::Scene("test_scene", URDF, SRDF);
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"continuous_joint", "revolute_joint"};

  // Build the conversion map
  auto conversion_map_maybe = buildConversionMap(scene, joint_state);
  ASSERT_TRUE(conversion_map_maybe.has_value());
  const auto conversion_map = conversion_map_maybe.value();

  // Confirm that there are two actuated joints of the correct type and order
  const auto& model = scene.getModel();
  ASSERT_EQ(conversion_map.mappings.size(), 2);
  EXPECT_EQ(conversion_map.nq, model.nq);
  EXPECT_EQ(conversion_map.nv, model.nv);
  EXPECT_EQ(conversion_map.mappings[0].type, roboplan::JointType::CONTINUOUS);
  EXPECT_EQ(conversion_map.mappings[1].type, roboplan::JointType::REVOLUTE);

  // Check each mapping
  for (size_t i = 0; i < conversion_map.mappings.size(); ++i) {
    const auto& mapping = conversion_map.mappings[i];
    EXPECT_EQ(mapping.joint_name, joint_state.name[i]);
    EXPECT_EQ(mapping.ros_index, i);
    const auto joint_id = model.getJointId(mapping.joint_name);
    EXPECT_EQ(mapping.q_start, model.idx_qs[joint_id]);
    EXPECT_EQ(mapping.v_start, model.idx_vs[joint_id]);
  }
}

TEST(TypeConversions, TestConvertJointState) {
  // Setup the Scene and ROS JointState message
  auto scene = roboplan::Scene("test_scene", URDF, SRDF);
  scene.setRngSeed(1234);
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = scene.getActuatedJointNames();

  // Build the conversion map
  auto conversion_map_maybe = buildConversionMap(scene, joint_state);
  ASSERT_TRUE(conversion_map_maybe.has_value());
  const auto conversion_map = conversion_map_maybe.value();

  // Setup a joint configuration
  roboplan::JointConfiguration joint_configuration;
  joint_configuration.joint_names = scene.getJointNames();
  joint_configuration.positions = scene.randomPositions();
  joint_configuration.velocities = Eigen::VectorXd::Zero(conversion_map.nv);
  joint_configuration.accelerations = Eigen::VectorXd::Zero(conversion_map.nv);

  // Convert back and forth and we should get the same values
  const auto ros_joint_state_maybe = roboplan_ros_cpp::toJointState(joint_configuration, scene);
  ASSERT_TRUE(ros_joint_state_maybe.has_value());
  const auto ros_joint_state = ros_joint_state_maybe.value();
  ASSERT_EQ(ros_joint_state.name.size(), 2);

  const auto check_joint_configuration_maybe =
      roboplan_ros_cpp::fromJointState(ros_joint_state, scene, conversion_map);
  if (!check_joint_configuration_maybe.has_value()) {
    std::cout << check_joint_configuration_maybe.error() << std::endl;
  }
  ASSERT_TRUE(check_joint_configuration_maybe.has_value());
  const auto check_joint_configuration = check_joint_configuration_maybe.value();

  ASSERT_EQ(joint_configuration.joint_names, check_joint_configuration.joint_names);
  ASSERT_TRUE(joint_configuration.positions.isApprox(check_joint_configuration.positions));
  ASSERT_TRUE(joint_configuration.velocities.isApprox(check_joint_configuration.velocities));
  ASSERT_TRUE(joint_configuration.accelerations.isApprox(check_joint_configuration.accelerations));
}

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

TEST(ConversionTest, TestTransformStampedConversion) {
  // Convert a TransformStamped to CartesianConfiguration then convert it back
  // and make sure it is the same.
  geometry_msgs::msg::TransformStamped original_transform;
  original_transform.header.frame_id = "world";
  original_transform.child_frame_id = "end_effector";
  original_transform.transform.translation.x = 1.2;
  original_transform.transform.translation.y = -0.5;
  original_transform.transform.translation.z = 3.7;

  // Normalized quaternion (pi/2, -pi/2, pi/4)
  original_transform.transform.rotation.w = 0.6532815;
  original_transform.transform.rotation.x = 0.2705981;
  original_transform.transform.rotation.y = -0.6532815;
  original_transform.transform.rotation.z = -0.2705979;

  const auto cartesian_config = fromTransformStamped(original_transform);
  const auto converted_transform = toTransformStamped(cartesian_config);

  EXPECT_EQ(original_transform.header.frame_id, converted_transform.header.frame_id);
  EXPECT_EQ(original_transform.child_frame_id, converted_transform.child_frame_id);

  EXPECT_FLOAT_EQ(original_transform.transform.translation.x,
                  converted_transform.transform.translation.x);
  EXPECT_FLOAT_EQ(original_transform.transform.translation.y,
                  converted_transform.transform.translation.y);
  EXPECT_FLOAT_EQ(original_transform.transform.translation.z,
                  converted_transform.transform.translation.z);

  EXPECT_FLOAT_EQ(original_transform.transform.rotation.w,
                  converted_transform.transform.rotation.w);
  EXPECT_FLOAT_EQ(original_transform.transform.rotation.x,
                  converted_transform.transform.rotation.x);
  EXPECT_FLOAT_EQ(original_transform.transform.rotation.y,
                  converted_transform.transform.rotation.y);
  EXPECT_FLOAT_EQ(original_transform.transform.rotation.z,
                  converted_transform.transform.rotation.z);
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

  EXPECT_FLOAT_EQ(original_pose.position.x, converted_pose.position.x);
  EXPECT_FLOAT_EQ(original_pose.position.y, converted_pose.position.y);
  EXPECT_FLOAT_EQ(original_pose.position.z, converted_pose.position.z);
  EXPECT_FLOAT_EQ(original_pose.orientation.w, converted_pose.orientation.w);
  EXPECT_FLOAT_EQ(original_pose.orientation.x, converted_pose.orientation.x);
  EXPECT_FLOAT_EQ(original_pose.orientation.y, converted_pose.orientation.y);
  EXPECT_FLOAT_EQ(original_pose.orientation.z, converted_pose.orientation.z);
}

}  // namespace roboplan_ros_cpp
