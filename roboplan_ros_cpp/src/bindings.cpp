#include <nanobind/nanobind.h>

#include <geometry_msgs/msg/pose.hpp>

#include <nanobind/eigen/dense.h>

#include "roboplan_ros_cpp/bindings.hpp"
#include "roboplan_ros_cpp/type_conversions.hpp"

namespace nb = nanobind;
using namespace roboplan_ros_cpp;

NB_MODULE(bindings, m) {
  m.doc() = "RoboPlan ROS Python bindings for C++ Type Conversions";

  m.def("pose_to_se3", [](nb::handle py_pose) {
    auto pose = pyToMsg<geometry_msgs::msg::Pose>(py_pose);
    return poseToSE3(pose);
  });

  m.def("se3_to_pose", [](const Eigen::Matrix4d& transform) {
    auto pose = se3ToPose(transform);
    return msgToPy(pose, nb::module_::import_("geometry_msgs.msg").attr("Pose"));
  });

  m.def("to_joint_trajectory", [](nb::handle py_roboplan_traj) {
    auto cpp_roboplan_traj = nb::cast<roboplan::JointTrajectory>(py_roboplan_traj);
    auto ros_traj = toJointTrajectory(cpp_roboplan_traj);
    return msgToPy(ros_traj, nb::module_::import_("trajectory_msgs.msg").attr("JointTrajectory"));
  });

  m.def("from_joint_trajectory", [](nb::handle py_ros_traj) {
    auto ros_traj = pyToMsg<trajectory_msgs::msg::JointTrajectory>(py_ros_traj);
    auto roboplan_traj = fromJointTrajectory(ros_traj);
    return nb::cast(roboplan_traj);
  });
}
