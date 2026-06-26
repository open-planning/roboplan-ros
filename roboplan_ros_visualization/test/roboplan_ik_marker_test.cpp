#include <gtest/gtest.h>

#include <Eigen/Core>

#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_cpp/type_conversions.hpp>
#include <roboplan_ros_visualization/roboplan_ik_marker.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan_ros_visualization {

static const std::string TWO_LINK_URDF = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
)";

static const std::string TWO_LINK_SRDF = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <group name="arm">
    <chain base_link="world" tip_link="link2"/>
  </group>
</robot>
)";

class RoboplanIkMarkerTest : public ::testing::Test {};

TEST_F(RoboplanIkMarkerTest, MakeInteractiveMarkerDefaultFrame) {
  auto scene_ = std::make_shared<roboplan::Scene>("test", TWO_LINK_URDF, TWO_LINK_SRDF);

  // Specify links even though it's a silly robot
  const std::string joint_group = "arm";
  const std::string base_link = "world";
  const std::string tip_link = "link2";

  const auto joint_group_info = scene_->getJointGroupInfo(joint_group);
  ASSERT_TRUE(joint_group_info.has_value());
  const auto q_indices = joint_group_info->q_indices;

  // This 2-DOF arm needs a lot of iterating...
  roboplan::SimpleIkOptions ik_options;
  ik_options.max_iters = 1000;

  // Setup IK solver and a solve function to pass to the iMarker
  auto simple_ik = std::make_shared<roboplan::SimpleIk>(
      std::const_pointer_cast<roboplan::Scene>(scene_), ik_options);
  auto ik_solve_fn = [scene_, simple_ik, q_indices, joint_group, base_link,
                      tip_link](const Eigen::Matrix4d& target_pose,
                                const Eigen::VectorXd& seed) -> std::optional<Eigen::VectorXd> {
    roboplan::CartesianConfiguration goal;
    goal.base_frame = base_link;
    goal.tip_frame = tip_link;
    goal.tform = target_pose;

    roboplan::JointConfiguration seed_jc;
    seed_jc.positions = seed(q_indices);

    roboplan::JointConfiguration solution;
    if (simple_ik->solveIk(goal, seed_jc, solution)) {
      return scene_->toFullJointPositions(joint_group, solution.positions);
    }
    return std::nullopt;
  };

  auto ik_ = std::make_unique<RoboplanIKMarker>(scene_, base_link, tip_link, ik_solve_fn);

  // Verify marker construction with default frame
  const auto marker = ik_->construct_imarker();
  EXPECT_EQ(marker.header.frame_id, "world");
  EXPECT_EQ(marker.name, "ik_target");
  EXPECT_FLOAT_EQ(marker.scale, 0.2f);
  EXPECT_EQ(marker.controls.size(), 7u);

  // Compute a reachable target by forward kinematics at a known configuration
  Eigen::VectorXd q_target = scene_->getCurrentJointPositions();
  q_target[0] = 0.5;
  q_target[1] = 0.5;
  const Eigen::Matrix4d fk_target = scene_->forwardKinematics(q_target, "link2");
  geometry_msgs::msg::Pose target = roboplan_ros_cpp::se3ToPose(fk_target);

  // Solve the IK using iMarker feedback
  visualization_msgs::msg::InteractiveMarkerFeedback feedback;
  feedback.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE;
  feedback.pose = target;
  const auto result = ik_->process_feedback(feedback);

  // Verify the result is non-trivial and close to the start
  ASSERT_TRUE(result.has_value());
  const Eigen::Matrix4d fk = scene_->forwardKinematics(*result, "link2");
  EXPECT_FALSE(result->isZero(1e-3));
  EXPECT_NEAR(fk(0, 3), target.position.x, 0.01);
  EXPECT_NEAR(fk(1, 3), target.position.y, 0.01);
  EXPECT_NEAR(fk(2, 3), target.position.z, 0.01);
}

}  // namespace roboplan_ros_visualization
