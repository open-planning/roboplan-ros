#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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

  nb::class_<JointStateConverterMap::JointMapping>(m, "JointMapping")
      .def_ro("joint_name", &JointStateConverterMap::JointMapping::joint_name)
      .def_ro("ros_index", &JointStateConverterMap::JointMapping::ros_index)
      .def_ro("q_start", &JointStateConverterMap::JointMapping::q_start)
      .def_ro("v_start", &JointStateConverterMap::JointMapping::v_start)
      .def_ro("type", &JointStateConverterMap::JointMapping::type);

  nb::class_<JointStateConverterMap>(m, "JointStateConverterMap")
      .def_ro("mappings", &JointStateConverterMap::mappings)
      .def_ro("nq", &JointStateConverterMap::nq)
      .def_ro("nv", &JointStateConverterMap::nv);

  m.def(
      "build_conversion_map",
      [](nb::handle py_scene, nb::handle py_joint_state) {
        auto scene = nb::cast<roboplan::Scene>(py_scene);
        auto joint_state = pyToCppMsg<sensor_msgs::msg::JointState>(py_joint_state);
        auto result = buildConversionMap(scene, joint_state);
        if (result.has_value()) {
          return nb::cast(result.value());
        } else {
          throw std::runtime_error(result.error());
        }
      },
      "py_scene"_a, "py_joint_state"_a,
      "Constructs a JointState conversion map given a RoboPlan Scene and JointState message.");

  m.def(
      "to_joint_state",
      [](nb::handle py_config, nb::handle py_scene) {
        auto config = nb::cast<roboplan::JointConfiguration>(py_config);
        auto scene = nb::cast<roboplan::Scene>(py_scene);
        auto result = toJointState(config, scene);
        if (result.has_value()) {
          return cppToPyMsg(result.value(),
                            nb::module_::import_("sensor_msgs.msg").attr("JointState"));
        } else {
          throw std::runtime_error(result.error());
        }
      },
      "py_config"_a, "py_scene"_a,
      "Converts a roboplan::JointConfiguration to a ROS 2 sensor_msgs::msg::JointState message.");

  m.def(
      "from_joint_state",
      [](nb::handle py_joint_state, nb::handle py_scene,
         const JointStateConverterMap& conversion_map) {
        auto joint_state = pyToCppMsg<sensor_msgs::msg::JointState>(py_joint_state);
        auto scene = nb::cast<roboplan::Scene>(py_scene);
        auto result = fromJointState(joint_state, scene, conversion_map);
        if (result.has_value()) {
          return nb::cast(result.value());
        } else {
          throw std::runtime_error(result.error());
        }
      },
      "py_joint_state"_a, "py_scene"_a, "conversion_map"_a,
      "Converts a ROS 2 sensor_msgs::msg::JointState message to a roboplan::JointConfiguration.");
}
