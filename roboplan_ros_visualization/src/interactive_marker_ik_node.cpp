#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan_ros_cpp/kinematics.hpp>
#include <roboplan_ros_visualization/interactive_marker_ik.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan_ros_visualization {

/// This is a demo node that I'm using for testing to see how this stuff works with ROS
/// visualization stuff. Not sure I like it yet... The distance from this to a move_group node feels
/// iffy.
class InteractiveMarkerIKNode : public rclcpp::Node {
public:
  InteractiveMarkerIKNode();
  void init();

private:
  void onIKSolved(const std::optional<Eigen::VectorXd>& joint_positions,
                  const geometry_msgs::msg::Pose& pose);

  void publishJointStates();

  std::shared_ptr<roboplan::Scene> scene_;
  std::shared_ptr<roboplan_ros_cpp::RoboPlanIK> ik_solver_;
  std::shared_ptr<InteractiveMarkerIK> imarker_ik_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::VectorXd latest_joint_positions_;

  std::string joint_group_;
  std::string base_frame_;
  std::string tip_frame_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXi q_indices_;
  std::string marker_namespace_;
};

InteractiveMarkerIKNode::InteractiveMarkerIKNode() : Node("interactive_marker_ik_node") {

  declare_parameter("robot_description", rclcpp::PARAMETER_STRING);
  declare_parameter("robot_description_semantic", rclcpp::PARAMETER_STRING);
  declare_parameter("robot_description_package", "roboplan_example_models");
  declare_parameter("yaml_config_path", "");
  declare_parameter("joint_group", "fr3_arm");
  declare_parameter("base_frame", "fr3_link0");
  declare_parameter("tip_frame", "fr3_hand");
  declare_parameter("marker_namespace", "interactive_ik_marker");
  declare_parameter("initial_joint_positions", std::vector<double>{});
  declare_parameter("max_ik_iterations", 100);
  declare_parameter("ik_step_size", 0.25);
  declare_parameter("joint_state_publish_rate", 10.0);

  std::string urdf_xml = get_parameter("robot_description").as_string();
  std::string srdf_xml = get_parameter("robot_description_semantic").as_string();
  std::string robot_description_package = get_parameter("robot_description_package").as_string();
  std::string yaml_config_path = get_parameter("yaml_config_path").as_string();
  joint_group_ = get_parameter("joint_group").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  tip_frame_ = get_parameter("tip_frame").as_string();
  marker_namespace_ = get_parameter("marker_namespace").as_string();
  auto initial_positions = get_parameter("initial_joint_positions").as_double_array();
  int max_ik_iterations = get_parameter("max_ik_iterations").as_int();
  double ik_step_size = get_parameter("ik_step_size").as_double();
  double joint_state_publish_rate = get_parameter("joint_state_publish_rate").as_double();

  // Setup RoboPlan Scene
  RCLCPP_INFO(get_logger(), "Creating RoboPlan scene...");
  std::vector<std::filesystem::path> package_paths = {std::filesystem::path(
      ament_index_cpp::get_package_share_directory(robot_description_package))};
  scene_ = std::make_shared<roboplan::Scene>("scene", urdf_xml, srdf_xml, package_paths,
                                             yaml_config_path);

  // Set initial joint positions if they are provided
  if (!initial_positions.empty()) {
    latest_joint_positions_ =
        Eigen::Map<Eigen::VectorXd>(initial_positions.data(), initial_positions.size());
  } else {
    latest_joint_positions_ = Eigen::VectorXd::Zero(scene_->getJointNames().size());
  }
  scene_->setJointPositions(latest_joint_positions_);

  // Get joint group information and joint names
  auto joint_group_info = scene_->getJointGroupInfo(joint_group_);
  if (!joint_group_info.has_value()) {
    RCLCPP_ERROR(get_logger(), "Failed to get joint group info: %s",
                 joint_group_info.error().c_str());
    throw std::runtime_error("Failed to get joint group info");
  }
  q_indices_ = joint_group_info.value().q_indices;

  const auto joint_names = scene_->getJointNames();
  for (int i = 0; i < q_indices_.size(); ++i) {
    joint_names_.push_back(joint_names[q_indices_(i)]);
  }

  // Publish joint states at a fixed rate
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  auto timer_period = std::chrono::duration<double>(1.0 / joint_state_publish_rate);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                             std::bind(&InteractiveMarkerIKNode::publishJointStates, this));

  // Set up the IK solver
  RCLCPP_INFO(get_logger(), "Setting up IK solver for group: %s", joint_group_.c_str());

  roboplan::SimpleIkOptions options;
  options.group_name = joint_group_;
  options.max_iters = max_ik_iterations;
  options.step_size = ik_step_size;
  ik_solver_ = std::make_shared<roboplan_ros_cpp::RoboPlanIK>(scene_, joint_group_, base_frame_,
                                                              tip_frame_, options);
}

void InteractiveMarkerIKNode::init() {
  // Create the interactive marker controller, which requires the node
  RCLCPP_INFO(get_logger(), "Creating interactive marker controller...");

  imarker_ik_ =
      std::make_shared<InteractiveMarkerIK>(shared_from_this(), scene_, ik_solver_,
                                            std::bind(&InteractiveMarkerIKNode::onIKSolved, this,
                                                      std::placeholders::_1, std::placeholders::_2),
                                            marker_namespace_);

  RCLCPP_INFO(get_logger(), "Interactive Marker IK Node initialized successfully!");
  RCLCPP_INFO(get_logger(), "Joint group: %s", joint_group_.c_str());
  RCLCPP_INFO(get_logger(), "Joints: %zu", joint_names_.size());
  RCLCPP_INFO(get_logger(), "End-effector: %s", tip_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Move the interactive marker in RViz to generate IK solutions");
}

void InteractiveMarkerIKNode::onIKSolved(const std::optional<Eigen::VectorXd>& joint_positions,
                                         const geometry_msgs::msg::Pose& /* pose */) {

  if (joint_positions.has_value()) {
    // Expand to full positions for publishing
    latest_joint_positions_(q_indices_) = joint_positions.value();
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(),
                         1000,  // 1 second throttle
                         "IK failed to find solution for target pose");
  }
}

void InteractiveMarkerIKNode::publishJointStates() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now();
  msg.header.frame_id = "world";
  msg.name = scene_->getJointNames();

  // Convert Eigen to std::vector
  msg.position.resize(latest_joint_positions_.size());
  for (int i = 0; i < latest_joint_positions_.size(); ++i) {
    msg.position[i] = latest_joint_positions_(i);
  }

  joint_state_pub_->publish(msg);
}

}  // namespace roboplan_ros_visualization

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roboplan_ros_visualization::InteractiveMarkerIKNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
