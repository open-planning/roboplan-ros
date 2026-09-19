#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_visualization/roboplan_visualizer.hpp>

namespace roboplan_ros_visualization {

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

// A two-link robot, each link carrying a distinct visual geometry, used to exercise filtering
// the published markers down to a single joint group.
static const std::string TWO_LINK_URDF = R"(<?xml version="1.0"?>
<robot name="two_link_robot">
  <link name="world"/>
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="link2">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
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
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
)";

// Defines a group containing only the first joint (which carries the second link along), a group
// containing only the second joint/link, plus a group that lists the second joint and additionally
// pulls in the first link via an explicit <link> element.
static const std::string TWO_LINK_SRDF = R"(<?xml version="1.0"?>
<robot name="two_link_robot">
  <group name="first">
    <joint name="joint1"/>
  </group>
  <group name="second">
    <joint name="joint2"/>
  </group>
  <group name="second_plus">
    <joint name="joint2"/>
    <link name="link1"/>
  </group>
</robot>
)";

static std::vector<decltype(visualization_msgs::msg::Marker::type)>
markerTypes(const std::vector<visualization_msgs::msg::Marker>& markers) {
  std::vector<decltype(visualization_msgs::msg::Marker::type)> types;
  for (const auto& marker : markers) {
    types.push_back(marker.type);
  }
  return types;
}

class RoboplanVisualizerTest : public ::testing::Test {};

TEST_F(RoboplanVisualizerTest, VisualizeFixedJointConfiguration) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(BOX_URDF));
  ASSERT_TRUE(scene->importSrdf(EMPTY_SRDF).has_value());
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
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(REVOLUTE_URDF));
  ASSERT_TRUE(scene->importSrdf(EMPTY_SRDF).has_value());
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

TEST_F(RoboplanVisualizerTest, VisualizeFullSceneByDefault) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(TWO_LINK_URDF));
  ASSERT_TRUE(scene->importSrdf(TWO_LINK_SRDF).has_value());
  RoboplanVisualizer viz(scene, TWO_LINK_URDF);

  const Eigen::VectorXd q = scene->getCurrentJointPositions();

  // With no group selected, both links' geometry should be rendered.
  const auto all_markers = viz.markers_from_configuration(q);
  ASSERT_EQ(all_markers.markers.size(), 2u);

  // Selecting the empty group explicitly is equivalent to the whole scene.
  viz.set_group("");
  ASSERT_EQ(viz.markers_from_configuration(q).markers.size(), 2u);
}

TEST_F(RoboplanVisualizerTest, VisualizeSingleJointGroupViaSetGroup) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(TWO_LINK_URDF));
  ASSERT_TRUE(scene->importSrdf(TWO_LINK_SRDF).has_value());
  RoboplanVisualizer viz(scene, TWO_LINK_URDF);

  const Eigen::VectorXd q = scene->getCurrentJointPositions();

  // The "second" group only drives link2, which carries the sphere geometry. The box on link1
  // sits above the group in the kinematic tree and does not move with it, so it is not drawn.
  viz.set_group("second");
  const auto second_markers = viz.markers_from_configuration(q);
  ASSERT_EQ(second_markers.markers.size(), 1u);
  EXPECT_EQ(second_markers.markers[0].type, visualization_msgs::msg::Marker::SPHERE);

  // Switching back to the whole scene takes effect immediately.
  viz.set_group("");
  ASSERT_EQ(viz.markers_from_configuration(q).markers.size(), 2u);
}

TEST_F(RoboplanVisualizerTest, VisualizeGroupCarriesDownstreamLinks) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(TWO_LINK_URDF));
  ASSERT_TRUE(scene->importSrdf(TWO_LINK_SRDF).has_value());
  // The "first" group only contains joint1, but link2 hangs below it and moves whenever joint1
  // does, so both geometries are rendered.
  RoboplanVisualizer viz(scene, TWO_LINK_URDF, "world", "/roboplan", "first");

  const Eigen::VectorXd q = scene->getCurrentJointPositions();
  const auto markers = viz.markers_from_configuration(q);
  ASSERT_EQ(markers.markers.size(), 2u);

  const auto types = markerTypes(markers.markers);
  EXPECT_NE(std::find(types.begin(), types.end(), visualization_msgs::msg::Marker::CUBE),
            types.end());
  EXPECT_NE(std::find(types.begin(), types.end(), visualization_msgs::msg::Marker::SPHERE),
            types.end());
}

TEST_F(RoboplanVisualizerTest, VisualizeGroupWithExplicitLink) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(TWO_LINK_URDF));
  ASSERT_TRUE(scene->importSrdf(TWO_LINK_SRDF).has_value());
  // The "second_plus" group drives link2 (sphere) and additionally lists link1 (box) explicitly,
  // so both geometries should be rendered.
  RoboplanVisualizer viz(scene, TWO_LINK_URDF, "world", "/roboplan", "second_plus");

  const Eigen::VectorXd q = scene->getCurrentJointPositions();
  const auto markers = viz.markers_from_configuration(q);
  ASSERT_EQ(markers.markers.size(), 2u);

  const auto types = markerTypes(markers.markers);
  EXPECT_NE(std::find(types.begin(), types.end(), visualization_msgs::msg::Marker::CUBE),
            types.end());
  EXPECT_NE(std::find(types.begin(), types.end(), visualization_msgs::msg::Marker::SPHERE),
            types.end());
}

TEST_F(RoboplanVisualizerTest, ConstructorGroupIsUsed) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(TWO_LINK_URDF));
  ASSERT_TRUE(scene->importSrdf(TWO_LINK_SRDF).has_value());
  // Configure the visualizer with the "second" group as its selection.
  RoboplanVisualizer viz(scene, TWO_LINK_URDF, "world", "/roboplan", "second");

  const Eigen::VectorXd q = scene->getCurrentJointPositions();

  // The constructor's group is used (link2, the sphere).
  const auto markers = viz.markers_from_configuration(q);
  ASSERT_EQ(markers.markers.size(), 1u);
  EXPECT_EQ(markers.markers[0].type, visualization_msgs::msg::Marker::SPHERE);
}

TEST_F(RoboplanVisualizerTest, UnknownGroupThrows) {
  const auto scene = std::make_shared<roboplan::Scene>(
      "test", roboplan::loadUrdfSceneDescriptionFromXml(TWO_LINK_URDF));
  ASSERT_TRUE(scene->importSrdf(TWO_LINK_SRDF).has_value());

  // An unknown group is rejected both at construction and via set_group.
  EXPECT_THROW(RoboplanVisualizer(scene, TWO_LINK_URDF, "world", "/roboplan", "does_not_exist"),
               std::runtime_error);

  RoboplanVisualizer viz(scene, TWO_LINK_URDF);
  EXPECT_THROW(viz.set_group("does_not_exist"), std::runtime_error);
}

}  // namespace roboplan_ros_visualization
