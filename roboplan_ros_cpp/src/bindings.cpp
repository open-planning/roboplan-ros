#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "roboplan_ros_cpp/bindings.hpp"
#include "roboplan_ros_cpp/type_conversions.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace roboplan_ros_cpp;

NB_MODULE(bindings, m) {
  m.doc() = "RoboPlan ROS Python bindings for C++ Type Conversions";

  m.def(
      "se3_to_pose",
      [](const Eigen::Matrix4d& transform) {
        auto pose = se3ToPose(transform);
        return cppToPyMsg(pose, nb::module_::import_("geometry_msgs.msg").attr("Pose"));
      },
      "transform"_a, "Converts an SE3 transformation matrix to a geometry_msgs::msg::Pose.");

  m.def(
      "pose_to_se3",
      [](nb::handle py_pose) {
        auto pose = pyToCppMsg<geometry_msgs::msg::Pose>(py_pose);
        return poseToSE3(pose);
      },
      "py_pose"_a, "Converts the provided geometry_msgs::msg::Pose to an SE3 Matrix.");

  m.def(
      "to_joint_trajectory",
      [](nb::handle py_roboplan_traj) {
        auto cpp_roboplan_traj = nb::cast<roboplan::JointTrajectory>(py_roboplan_traj);
        auto ros_traj = toJointTrajectory(cpp_roboplan_traj);
        return cppToPyMsg(ros_traj,
                          nb::module_::import_("trajectory_msgs.msg").attr("JointTrajectory"));
      },
      "py_roboplan_traj"_a,
      "Converts a roboplan::JointTrajectory to a trajectory_msgs::msg::JointTrajectory.");

  m.def(
      "from_joint_trajectory",
      [](nb::handle py_ros_traj) {
        auto ros_traj = pyToCppMsg<trajectory_msgs::msg::JointTrajectory>(py_ros_traj);
        auto roboplan_traj = fromJointTrajectory(ros_traj);
        return nb::cast(roboplan_traj);
      },
      "py_ros_traj"_a,
      "Converts a trajectory_msgs::msg::JointTrajectory to a roboplan::JointTrajectory.");

  m.def(
      "to_transform_stamped",
      [](const roboplan::CartesianConfiguration& cartesian_config) {
        auto transform = toTransformStamped(cartesian_config);
        return cppToPyMsg(transform,
                          nb::module_::import_("geometry_msgs.msg").attr("TransformStamped"));
      },
      "cartesian_config"_a,
      "Converts a roboplan::CartesianConfiguration to a geometry_msgs::msg::TransformStamped.");

  m.def(
      "from_transform_stamped",
      [](nb::handle py_transform) {
        auto transform = pyToCppMsg<geometry_msgs::msg::TransformStamped>(py_transform);
        auto cartesian_config = fromTransformStamped(transform);
        return nb::cast(cartesian_config);
      },
      "py_transform"_a,
      "Converts a geometry_msgs::msg::TransformStamped to a roboplan::CartesianConfiguration.");
}
