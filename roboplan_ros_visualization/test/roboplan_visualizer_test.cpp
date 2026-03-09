#include <gtest/gtest.h>

#include <Eigen/Core>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_visualization/roboplan_visualizer.hpp>

namespace roboplan_ros_cpp {

static const std::string BOX_URDF = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <link name="box_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
  <joint name="fixed_joint" type="fixed">
    <parent link="world"/>
    <child link="box_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
</robot>
)";

static const std::string REVOLUTE_URDF = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
)";

static const std::string EMPTY_SRDF = R"(<?xml version="1.0"?>
<robot name="test_robot">
</robot>
)";

class RoboplanVisualizerTest : public ::testing::Test {};

TEST_F(RoboplanVisualizerTest, VisualizeFixedJointConfiguration) {
  const auto scene = std::make_shared<roboplan::Scene>("test", BOX_URDF, EMPTY_SRDF);
  RoboplanVisualizer viz(scene, BOX_URDF);

  const Eigen::VectorXd q = scene->getCurrentJointPositions();
  const auto marker_array = viz.markers_from_configuration(q);

  // One visual geometry (the box) should produce one marker
  ASSERT_EQ(marker_array.markers.size(), 1u);
  EXPECT_EQ(marker_array.markers[0].type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_DOUBLE_EQ(marker_array.markers[0].scale.x, 0.1);
  EXPECT_DOUBLE_EQ(marker_array.markers[0].scale.y, 0.1);
  EXPECT_DOUBLE_EQ(marker_array.markers[0].scale.z, 0.1);
}

TEST_F(RoboplanVisualizerTest, VisualizeRevoluteJointConfiguration) {
  const auto scene = std::make_shared<roboplan::Scene>("test", REVOLUTE_URDF, EMPTY_SRDF);
  RoboplanVisualizer viz(scene, REVOLUTE_URDF);

  const Eigen::VectorXd q = scene->getCurrentJointPositions();
  const auto marker_array = viz.markers_from_configuration(q);

  ASSERT_EQ(marker_array.markers.size(), 1u);
  EXPECT_EQ(marker_array.markers[0].type, visualization_msgs::msg::Marker::CYLINDER);
  EXPECT_DOUBLE_EQ(marker_array.markers[0].scale.x, 0.1);  // radius * 2
  EXPECT_DOUBLE_EQ(marker_array.markers[0].scale.y, 0.1);
  EXPECT_DOUBLE_EQ(marker_array.markers[0].scale.z, 0.5);  // halfLength * 2

  const auto marker_array_new = viz.clear_markers();
  ASSERT_EQ(marker_array_new.markers.size(), 1u);
  EXPECT_EQ(marker_array_new.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
}

}  // namespace roboplan_ros_cpp
