#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_visualization/interactive_marker_ik.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("interactive_marker_ik_node");

  // Parameter defaults hardcoded to the franka example...
  node->declare_parameter("robot_description", rclcpp::PARAMETER_STRING);
  node->declare_parameter("robot_description_semantic", rclcpp::PARAMETER_STRING);
  node->declare_parameter("robot_description_package", "roboplan_example_models");
  node->declare_parameter("yaml_config_path", "");
  node->declare_parameter("joint_group", "fr3_arm");
  node->declare_parameter("base_frame", "fr3_link0");
  node->declare_parameter("tip_frame", "fr3_hand");
  node->declare_parameter("marker_namespace", "roboplan_ik");
  node->declare_parameter("initial_joint_positions", std::vector<double>{});
  node->declare_parameter("max_ik_iterations", 100);
  node->declare_parameter("ik_step_size", 0.25);
  node->declare_parameter("check_collisions", true);

  std::string urdf_xml = node->get_parameter("robot_description").as_string();
  std::string srdf_xml = node->get_parameter("robot_description_semantic").as_string();
  std::string robot_description_package =
      node->get_parameter("robot_description_package").as_string();
  std::string yaml_config_path = node->get_parameter("yaml_config_path").as_string();
  std::string joint_group = node->get_parameter("joint_group").as_string();
  std::string base_frame = node->get_parameter("base_frame").as_string();
  std::string tip_frame = node->get_parameter("tip_frame").as_string();
  std::string marker_namespace = node->get_parameter("marker_namespace").as_string();
  auto initial_positions = node->get_parameter("initial_joint_positions").as_double_array();
  int max_ik_iterations = node->get_parameter("max_ik_iterations").as_int();
  double ik_step_size = node->get_parameter("ik_step_size").as_double();
  bool check_collisions = node->get_parameter("check_collisions").as_bool();

  // Initialize the RoboPlan scene
  RCLCPP_INFO(node->get_logger(), "Creating RoboPlan scene...");
  std::vector<std::filesystem::path> package_paths = {std::filesystem::path(
      ament_index_cpp::get_package_share_directory(robot_description_package))};
  auto scene = std::make_shared<roboplan::Scene>("scene", urdf_xml, srdf_xml, package_paths,
                                                 yaml_config_path);

  // Set initial positions
  if (!initial_positions.empty()) {
    Eigen::VectorXd positions =
        Eigen::Map<Eigen::VectorXd>(initial_positions.data(), initial_positions.size());
    scene->setJointPositions(positions);
  }

  // Setup IK options
  roboplan::SimpleIkOptions options;
  options.group_name = joint_group;
  options.max_iters = max_ik_iterations;
  options.step_size = ik_step_size;
  options.check_collisions = check_collisions;

  // Create interactive marker controller and spin
  RCLCPP_INFO(node->get_logger(), "Creating interactive marker controller...");
  auto imarker_ik = std::make_shared<roboplan_ros_visualization::InteractiveMarkerIK>(
      node, scene, joint_group, base_frame, tip_frame, options, marker_namespace);
  RCLCPP_INFO(node->get_logger(), "Ready! Move the interactive marker in RViz.");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
