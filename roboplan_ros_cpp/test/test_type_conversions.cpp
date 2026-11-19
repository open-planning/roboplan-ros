#include <filesystem>
#include <numbers>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <ament_index_cpp/version.h>
#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 2)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_cpp {

class TypeConversionsTest : public ::testing::Test {
protected:
  void SetUp() override {
#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 2)
    const auto resources_dir =
        ament_index_cpp::get_package_share_path("roboplan_ros_cpp") / "test" / "resources";
#else
    const auto resources_dir =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("roboplan_ros_cpp")) /
        "test" / "resources";
#endif

    urdf_path = resources_dir / "test_robot.urdf";
    srdf_path = resources_dir / "test_robot.srdf";
    mobile_urdf_path = resources_dir / "test_mobile_robot.urdf";
  }

  std::filesystem::path urdf_path;
  std::filesystem::path srdf_path;
  std::filesystem::path mobile_urdf_path;
};

TEST_F(TypeConversionsTest, TestJointStateMapping) {
  // Setup the Scene and ROS JointState message
  auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));
  ASSERT_TRUE(scene.importSrdf(roboplan::loadTextFile(srdf_path)).has_value());
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

TEST_F(TypeConversionsTest, TestConvertJointState) {
  // Setup the Scene and ROS JointState message
  auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));
  ASSERT_TRUE(scene.importSrdf(roboplan::loadTextFile(srdf_path)).has_value());
  scene.setRngSeed(1234);
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = scene.getJointNames();

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
  ASSERT_EQ(ros_joint_state.name.size(), 2u);

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

TEST_F(TypeConversionsTest, TestConvertPartialJointState) {
  auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));
  ASSERT_TRUE(scene.importSrdf(roboplan::loadTextFile(srdf_path)).has_value());
  scene.setRngSeed(1234);
  const Eigen::VectorXd q_current = scene.randomPositions();
  scene.setJointPositions(q_current);

  // The message has the revolute joint only, not the continuous one.
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"revolute_joint"};
  joint_state.position = {0.3};
  const auto conversion_map_maybe = buildConversionMap(scene, joint_state);
  ASSERT_TRUE(conversion_map_maybe.has_value());
  const auto conversion_map = conversion_map_maybe.value();
  const auto continuous_idx = scene.getJointPositionIndices({"continuous_joint"});
  const auto revolute_idx = scene.getJointPositionIndices({"revolute_joint"});

  // Without a reference, the missing joint keeps the scene's current position.
  const auto from_current = fromJointState(joint_state, scene, conversion_map);
  ASSERT_TRUE(from_current.has_value()) << from_current.error();
  EXPECT_TRUE(from_current->positions(continuous_idx).isApprox(q_current(continuous_idx)));
  EXPECT_DOUBLE_EQ(from_current->positions(revolute_idx(0)), 0.3);

  // With a reference, it keeps the reference's position instead.
  const Eigen::VectorXd q_reference = scene.randomPositions();
  const auto from_reference = fromJointState(joint_state, scene, conversion_map, q_reference);
  ASSERT_TRUE(from_reference.has_value()) << from_reference.error();
  EXPECT_TRUE(from_reference->positions(continuous_idx).isApprox(q_reference(continuous_idx)));
  EXPECT_DOUBLE_EQ(from_reference->positions(revolute_idx(0)), 0.3);

  // A reference of the wrong size is an error.
  const auto wrong_size =
      fromJointState(joint_state, scene, conversion_map, Eigen::VectorXd::Zero(1));
  ASSERT_FALSE(wrong_size.has_value());
  const auto nq = std::to_string(scene.getModel().nq);
  EXPECT_EQ(wrong_size.error(), "q_reference has 1 positions, the scene has " + nq);
}

TEST_F(TypeConversionsTest, TestConvertJointTrajectoryWithContinuousJoint) {
  const auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));

  // RoboPlan stores the continuous joint as [cos(theta), sin(theta)], ROS as theta.
  const auto make_q = [](double theta, double revolute) {
    return (Eigen::VectorXd(3) << std::cos(theta), std::sin(theta), revolute).finished();
  };
  const auto make_v = [](double a, double b) { return (Eigen::VectorXd(2) << a, b).finished(); };
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = {"continuous_joint", "revolute_joint"};
  roboplan_traj.times = {0.0, 1.0};
  roboplan_traj.positions = {make_q(0.5, 0.1), make_q(-2.0, 0.2)};
  roboplan_traj.velocities = {make_v(0.0, 0.0), make_v(1.0, -1.0)};
  roboplan_traj.accelerations = {make_v(0.0, 0.0), make_v(2.0, -2.0)};

  const auto ros_traj = toJointTrajectory(roboplan_traj, scene);
  ASSERT_TRUE(ros_traj.has_value()) << ros_traj.error();
  ASSERT_EQ(ros_traj->joint_names, roboplan_traj.joint_names);
  ASSERT_EQ(ros_traj->points.size(), 2u);
  ASSERT_EQ(ros_traj->points[1].positions.size(), 2u);
  EXPECT_DOUBLE_EQ(ros_traj->points[1].positions[0], -2.0);
  EXPECT_DOUBLE_EQ(ros_traj->points[1].positions[1], 0.2);
  EXPECT_EQ(ros_traj->points[1].velocities, (std::vector<double>{1.0, -1.0}));

  // Converting it back gives the original trajectory.
  const auto new_roboplan_traj = fromJointTrajectory(ros_traj.value(), scene);
  ASSERT_TRUE(new_roboplan_traj.has_value()) << new_roboplan_traj.error();
  EXPECT_EQ(new_roboplan_traj->joint_names, roboplan_traj.joint_names);
  EXPECT_EQ(new_roboplan_traj->times, roboplan_traj.times);
  for (size_t i = 0; i < roboplan_traj.positions.size(); ++i) {
    EXPECT_TRUE(new_roboplan_traj->positions[i].isApprox(roboplan_traj.positions[i]));
    EXPECT_TRUE(new_roboplan_traj->velocities[i].isApprox(roboplan_traj.velocities[i]));
    EXPECT_TRUE(new_roboplan_traj->accelerations[i].isApprox(roboplan_traj.accelerations[i]));
  }
}

TEST_F(TypeConversionsTest, TestConvertJointTrajectoryWithMimicJoint) {
  const auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));

  // A RoboPlan trajectory names the mimic joint, but has no values for it.
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = {"revolute_joint", "mimic_joint"};
  roboplan_traj.times = {0.0};
  roboplan_traj.positions = {Eigen::VectorXd::Constant(1, 0.3)};
  const auto ros_traj = toJointTrajectory(roboplan_traj, scene);
  ASSERT_TRUE(ros_traj.has_value()) << ros_traj.error();
  EXPECT_EQ(ros_traj->joint_names, std::vector<std::string>{"revolute_joint"});
  EXPECT_EQ(ros_traj->points[0].positions, std::vector<double>{0.3});

  // From ROS, the mimic joint's value is ignored.
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = {"revolute_joint", "mimic_joint"};
  msg.points.resize(1);
  msg.points[0].positions = {0.3, 99.0};
  const auto from_ros = fromJointTrajectory(msg, scene);
  ASSERT_TRUE(from_ros.has_value()) << from_ros.error();
  EXPECT_EQ(from_ros->joint_names, msg.joint_names);
  EXPECT_TRUE(from_ros->positions[0].isApprox(Eigen::VectorXd::Constant(1, 0.3)));
  EXPECT_EQ(from_ros->velocities[0].size(), 0);
}

TEST_F(TypeConversionsTest, TestConvertJointTrajectoryErrors) {
  const auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));

  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = {"continuous_joint", "revolute_joint"};
  msg.points.resize(1);
  msg.points[0].positions = {0.5};
  const auto wrong_size = fromJointTrajectory(msg, scene);
  ASSERT_FALSE(wrong_size.has_value());
  EXPECT_EQ(wrong_size.error(),
            "Failed to convert the trajectory: Point 0 has 1 positions, expected 2");

  msg.joint_names = {"continuous_joint", "no_such_joint"};
  msg.points[0].positions = {0.5, 0.1};
  EXPECT_FALSE(fromJointTrajectory(msg, scene).has_value());

  // RoboPlan has two positions for the continuous joint.
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = {"continuous_joint"};
  roboplan_traj.times = {0.0};
  roboplan_traj.positions = {Eigen::VectorXd::Constant(1, 0.5)};
  const auto collapsed = toJointTrajectory(roboplan_traj, scene);
  ASSERT_FALSE(collapsed.has_value());
  EXPECT_EQ(collapsed.error(),
            "Failed to convert the trajectory: Point 0 has 1 positions, expected 2");
}

TEST_F(TypeConversionsTest, TestJointTrajectoryUnwrapsContinuousJoints) {
  const auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));

  // The continuous joint crosses +-pi between the first two points.
  const auto make_q = [](double theta) {
    return (Eigen::VectorXd(2) << std::cos(theta), std::sin(theta)).finished();
  };
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = {"continuous_joint"};
  roboplan_traj.times = {0.0, 1.0, 2.0};
  roboplan_traj.positions = {make_q(3.0), make_q(-3.0), make_q(-2.5)};

  // In ROS the angle keeps going past pi instead of jumping to -pi.
  const auto ros_traj = toJointTrajectory(roboplan_traj, scene);
  ASSERT_TRUE(ros_traj.has_value()) << ros_traj.error();
  const double two_pi = 2.0 * std::numbers::pi;
  EXPECT_NEAR(ros_traj->points[0].positions[0], 3.0, 1e-12);
  EXPECT_NEAR(ros_traj->points[1].positions[0], two_pi - 3.0, 1e-12);
  EXPECT_NEAR(ros_traj->points[2].positions[0], two_pi - 2.5, 1e-12);

  const auto new_roboplan_traj = fromJointTrajectory(ros_traj.value(), scene);
  ASSERT_TRUE(new_roboplan_traj.has_value()) << new_roboplan_traj.error();
  for (size_t i = 0; i < roboplan_traj.positions.size(); ++i) {
    EXPECT_TRUE(new_roboplan_traj->positions[i].isApprox(roboplan_traj.positions[i]));
  }
}

TEST_F(TypeConversionsTest, TestConvertMultiDOFJointTrajectory) {
  const auto scene =
      roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(mobile_urdf_path));
  const std::vector<std::string> joint_names = {"base_joint", "wheel_joint", "drone_joint"};
  ASSERT_EQ(scene.getJointNames(), joint_names);

  // Planar [x, y, cos, sin], continuous [cos, sin], floating [x, y, z, qx, qy, qz, qw]. The base
  // turns from 3 to -3 rad, across +-pi.
  const auto make_q = [](double base_theta, double drone_z) {
    const Eigen::Quaterniond drone_rotation(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));
    Eigen::VectorXd q(13);
    q << 1.0, 2.0, std::cos(base_theta), std::sin(base_theta), std::cos(0.5), std::sin(0.5), 0.1,
        0.2, drone_z, drone_rotation.x(), drone_rotation.y(), drone_rotation.z(),
        drone_rotation.w();
    return q;
  };
  Eigen::VectorXd v(10);
  v << 0.1, 0.2, 0.3, 0.4, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = joint_names;
  roboplan_traj.times = {0.0, 1.0};
  roboplan_traj.positions = {make_q(3.0, 1.0), make_q(-3.0, 1.5)};
  roboplan_traj.velocities = {v, v};
  roboplan_traj.accelerations = {2.0 * v, 2.0 * v};

  // A JointTrajectory alone cannot hold the base and the drone.
  const auto joint_only = toJointTrajectory(roboplan_traj, scene);
  ASSERT_FALSE(joint_only.has_value());
  EXPECT_EQ(joint_only.error(),
            "Failed to convert the trajectory: Joint 'base_joint' has several degrees of freedom: "
            "a JointTrajectory cannot represent it, use toRosTrajectories");

  const auto ros_trajs = toRosTrajectories(roboplan_traj, scene);
  ASSERT_TRUE(ros_trajs.has_value()) << ros_trajs.error();
  const auto& [ros_traj, multi_dof_traj] = ros_trajs.value();

  // The JointTrajectory only has the continuous joint.
  EXPECT_EQ(ros_traj.joint_names, std::vector<std::string>{"wheel_joint"});
  EXPECT_NEAR(ros_traj.points[0].positions[0], 0.5, 1e-12);
  EXPECT_EQ(ros_traj.points[0].velocities, std::vector<double>{0.4});

  // The MultiDOFJointTrajectory has the planar and floating joints.
  EXPECT_EQ(multi_dof_traj.joint_names, (std::vector<std::string>{"base_joint", "drone_joint"}));
  ASSERT_EQ(multi_dof_traj.points.size(), 2u);
  const auto& base = multi_dof_traj.points[0].transforms[0];
  EXPECT_DOUBLE_EQ(base.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(base.translation.y, 2.0);
  EXPECT_NEAR(base.rotation.z, std::sin(1.5), 1e-12);
  EXPECT_NEAR(base.rotation.w, std::cos(1.5), 1e-12);
  const auto& drone = multi_dof_traj.points[1].transforms[1];
  EXPECT_DOUBLE_EQ(drone.translation.z, 1.5);
  EXPECT_NEAR(drone.rotation.x, std::sin(0.2), 1e-12);

  // At -3 rad the base's quaternion keeps the sign of the previous point: it is the rotation by
  // 2 pi - 3, so interpolating does not turn the base the long way round.
  const auto& turned_base = multi_dof_traj.points[1].transforms[0];
  EXPECT_NEAR(turned_base.rotation.z, std::sin((2.0 * std::numbers::pi - 3.0) / 2.0), 1e-12);
  EXPECT_NEAR(turned_base.rotation.w, std::cos((2.0 * std::numbers::pi - 3.0) / 2.0), 1e-12);

  // Twists are in the joint's frame, as in RoboPlan.
  const auto& base_twist = multi_dof_traj.points[0].velocities[0];
  EXPECT_DOUBLE_EQ(base_twist.linear.x, 0.1);
  EXPECT_DOUBLE_EQ(base_twist.linear.y, 0.2);
  EXPECT_DOUBLE_EQ(base_twist.angular.z, 0.3);
  const auto& drone_acceleration = multi_dof_traj.points[0].accelerations[1];
  EXPECT_DOUBLE_EQ(drone_acceleration.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(drone_acceleration.angular.z, 12.0);

  // Converting both messages back gives the original trajectory, with the joints of the
  // JointTrajectory first: the wheel, then the base and the drone.
  const auto new_roboplan_traj = fromJointTrajectory(ros_traj, multi_dof_traj, scene);
  ASSERT_TRUE(new_roboplan_traj.has_value()) << new_roboplan_traj.error();
  EXPECT_EQ(new_roboplan_traj->joint_names,
            (std::vector<std::string>{"wheel_joint", "base_joint", "drone_joint"}));
  EXPECT_EQ(new_roboplan_traj->times, roboplan_traj.times);
  // Moves the wheel's values, after the base's base_size ones, to the front.
  const auto wheel_first = [](const Eigen::VectorXd& x, int base_size, int wheel_size) {
    Eigen::VectorXd y(x.size());
    y << x.segment(base_size, wheel_size), x.head(base_size),
        x.tail(x.size() - base_size - wheel_size);
    return y;
  };
  for (size_t i = 0; i < roboplan_traj.positions.size(); ++i) {
    EXPECT_TRUE(
        new_roboplan_traj->positions[i].isApprox(wheel_first(roboplan_traj.positions[i], 4, 2)));
    EXPECT_TRUE(
        new_roboplan_traj->velocities[i].isApprox(wheel_first(roboplan_traj.velocities[i], 3, 1)));
    EXPECT_TRUE(new_roboplan_traj->accelerations[i].isApprox(
        wheel_first(roboplan_traj.accelerations[i], 3, 1)));
  }

  // A trajectory without planar or floating joints gives an empty MultiDOFJointTrajectory.
  roboplan::JointTrajectory wheel_traj;
  wheel_traj.joint_names = {"wheel_joint"};
  wheel_traj.times = {0.0};
  wheel_traj.positions = {(Eigen::VectorXd(2) << 1.0, 0.0).finished()};
  const auto wheel_trajs = toRosTrajectories(wheel_traj, scene);
  ASSERT_TRUE(wheel_trajs.has_value()) << wheel_trajs.error();
  EXPECT_EQ(wheel_trajs->first.joint_names, std::vector<std::string>{"wheel_joint"});
  EXPECT_TRUE(wheel_trajs->second.joint_names.empty());
  EXPECT_TRUE(wheel_trajs->second.points.empty());
}

TEST_F(TypeConversionsTest, TestConvertMultiDOFJointTrajectoryErrors) {
  const auto scene =
      roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(mobile_urdf_path));
  const auto prefix = std::string("Failed to convert the trajectory: ");

  trajectory_msgs::msg::JointTrajectory joint_msg;
  joint_msg.joint_names = {"wheel_joint"};
  joint_msg.points.resize(2);
  joint_msg.points[0].positions = {0.5};
  joint_msg.points[1].positions = {0.6};
  joint_msg.points[1].time_from_start.sec = 1;

  trajectory_msgs::msg::MultiDOFJointTrajectory multi_dof_msg;
  multi_dof_msg.joint_names = {"base_joint"};
  multi_dof_msg.points.resize(2);
  for (auto& point : multi_dof_msg.points) {
    point.transforms.resize(1);
  }
  multi_dof_msg.points[1].time_from_start.sec = 1;
  ASSERT_TRUE(fromJointTrajectory(joint_msg, multi_dof_msg, scene).has_value());

  // A planar joint cannot be in a JointTrajectory, nor a continuous one in a
  // MultiDOFJointTrajectory.
  auto wrong_joint_msg = joint_msg;
  wrong_joint_msg.joint_names = {"base_joint"};
  EXPECT_EQ(fromJointTrajectory(wrong_joint_msg, scene).error(),
            prefix + "Joint 'base_joint' has several degrees of freedom: a JointTrajectory cannot "
                     "represent it, use a MultiDOFJointTrajectory");
  auto wrong_multi_dof_msg = multi_dof_msg;
  wrong_multi_dof_msg.joint_names = {"wheel_joint"};
  EXPECT_EQ(fromJointTrajectory(joint_msg, wrong_multi_dof_msg, scene).error(),
            prefix + "Joint 'wheel_joint' has a single degree of freedom: it belongs in the "
                     "JointTrajectory");

  // Both messages must have the same points.
  auto short_msg = multi_dof_msg;
  short_msg.points.pop_back();
  EXPECT_EQ(fromJointTrajectory(joint_msg, short_msg, scene).error(),
            prefix + "The JointTrajectory has 2 points, the MultiDOFJointTrajectory 1");
  auto late_msg = multi_dof_msg;
  late_msg.points[1].time_from_start.sec = 2;
  EXPECT_EQ(
      fromJointTrajectory(joint_msg, late_msg, scene).error(),
      prefix +
          "Point 1 has different times in the JointTrajectory and the MultiDOFJointTrajectory");

  // Velocities are given for all the joints or for none.
  auto moving_msg = multi_dof_msg;
  moving_msg.points[0].velocities.resize(1);
  EXPECT_EQ(fromJointTrajectory(joint_msg, moving_msg, scene).error(),
            prefix + "Point 0 has velocities for some joints only");

  // A quaternion must not be zero.
  auto zero_msg = multi_dof_msg;
  zero_msg.points[0].transforms[0].rotation.w = 0.0;
  EXPECT_EQ(fromJointTrajectory(joint_msg, zero_msg, scene).error(),
            prefix + "Point 0 has a zero quaternion for joint 'base_joint'");
}

TEST_F(TypeConversionsTest, TestEmptyConversions) {
  const auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));

  roboplan::JointTrajectory roboplan_traj;
  const auto ros_traj = toJointTrajectory(roboplan_traj, scene);
  ASSERT_TRUE(ros_traj.has_value()) << ros_traj.error();
  EXPECT_TRUE(ros_traj->joint_names.empty());

  const auto orig_roboplan_traj = fromJointTrajectory(ros_traj.value(), scene);
  ASSERT_TRUE(orig_roboplan_traj.has_value()) << orig_roboplan_traj.error();
  EXPECT_TRUE(orig_roboplan_traj->joint_names.empty());
}

TEST_F(TypeConversionsTest, TestConvertToJointTrajectory) {
  const auto scene = roboplan::Scene("test_scene", roboplan::loadUrdfSceneDescription(urdf_path));

  // Shorthand helper function
  auto make_vector = [](double a) { return Eigen::VectorXd::Constant(1, a); };

  // Create a roboplan trajectory with a revolute joint and two points
  roboplan::JointTrajectory roboplan_traj;
  roboplan_traj.joint_names = {"revolute_joint"};
  roboplan_traj.times = {0.0, 1.0};
  roboplan_traj.positions = {make_vector(0.0), make_vector(1.0)};
  roboplan_traj.velocities = {make_vector(0.0), make_vector(2.0)};
  roboplan_traj.accelerations = {make_vector(0.0), make_vector(3.0)};

  // Convert it and check the result, worrying about exact values below
  const auto ros_traj = toJointTrajectory(roboplan_traj, scene);
  ASSERT_TRUE(ros_traj.has_value()) << ros_traj.error();
  ASSERT_EQ(ros_traj->joint_names, roboplan_traj.joint_names);
  ASSERT_EQ(ros_traj->points.size(), 2);

  // Convert it back and we should get the original trajectory
  const auto new_roboplan_traj = fromJointTrajectory(ros_traj.value(), scene);
  ASSERT_TRUE(new_roboplan_traj.has_value()) << new_roboplan_traj.error();
  ASSERT_EQ(new_roboplan_traj->joint_names, roboplan_traj.joint_names);
  ASSERT_EQ(new_roboplan_traj->times, roboplan_traj.times);
  ASSERT_EQ(new_roboplan_traj->positions, roboplan_traj.positions);
  ASSERT_EQ(new_roboplan_traj->velocities, roboplan_traj.velocities);
  ASSERT_EQ(new_roboplan_traj->accelerations, roboplan_traj.accelerations);
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
