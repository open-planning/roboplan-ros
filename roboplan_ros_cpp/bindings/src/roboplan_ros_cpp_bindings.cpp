#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <roboplan_bindings/expected.hpp>

#include "roboplan_ros_cpp/type_conversions.hpp"
#include "roboplan_ros_cpp_bindings/roboplan_ros_cpp_bindings.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace roboplan_ros_cpp;

NB_MODULE(_cpp_ext, m) {
  m.doc() = "RoboPlan ROS nanobind wrappers for core C++ ROS tools.";

  m.def(
      "se3ToPose",
      [](const Eigen::Matrix4d& transform) {
        auto pose = se3ToPose(transform);
        return cppToPyMsg(pose, nb::module_::import_("geometry_msgs.msg").attr("Pose"));
      },
      "transform"_a, "Converts an SE3 transformation matrix to a geometry_msgs::msg::Pose.");

  m.def(
      "poseToSE3",
      [](nb::handle py_pose) {
        auto pose = pyToCppMsg<geometry_msgs::msg::Pose>(py_pose);
        return poseToSE3(pose);
      },
      "py_pose"_a, "Converts the provided geometry_msgs::msg::Pose to an SE3 Matrix.");

  // The trajectory conversions are overloaded as in C++, each overload with its own docstring.
  const auto joint_trajectory_class = [] {
    return nb::module_::import_("trajectory_msgs.msg").attr("JointTrajectory");
  };

  m.def(
      "toJointTrajectory",
      [=](nb::handle py_roboplan_traj) {
        auto ros_traj = toJointTrajectory(nb::cast<roboplan::JointTrajectory>(py_roboplan_traj));
        return cppToPyMsg(ros_traj, joint_trajectory_class());
      },
      "py_roboplan_traj"_a,
      "Converts a roboplan::JointTrajectory to a trajectory_msgs::msg::JointTrajectory. Positions "
      "are copied as they are, which is only correct without continuous joints; use the overload "
      "that takes a scene otherwise.");

  m.def(
      "toJointTrajectory",
      [=](nb::handle py_roboplan_traj, const roboplan::Scene& py_scene) {
        auto ros_traj = handle_expected(
            toJointTrajectory(nb::cast<roboplan::JointTrajectory>(py_roboplan_traj), py_scene));
        return cppToPyMsg(ros_traj, joint_trajectory_class());
      },
      "py_roboplan_traj"_a, "py_scene"_a,
      "Converts the single degree of freedom joints of a roboplan::JointTrajectory to a "
      "trajectory_msgs::msg::JointTrajectory. Continuous joints become angles, unwrapped along the "
      "trajectory. Mimic joints are left out, and so are planar and floating joints: use "
      "toMultiDOFJointTrajectory for them.");

  m.def(
      "toMultiDOFJointTrajectory",
      [](nb::handle py_roboplan_traj, const roboplan::Scene& py_scene) {
        auto ros_traj = handle_expected(toMultiDOFJointTrajectory(
            nb::cast<roboplan::JointTrajectory>(py_roboplan_traj), py_scene));
        return cppToPyMsg(
            ros_traj, nb::module_::import_("trajectory_msgs.msg").attr("MultiDOFJointTrajectory"));
      },
      "py_roboplan_traj"_a, "py_scene"_a,
      "Converts the planar and floating joints of a roboplan::JointTrajectory to a "
      "trajectory_msgs::msg::MultiDOFJointTrajectory, with twists in each joint's own frame.");

  m.def(
      "fromJointTrajectory",
      [](nb::handle py_ros_traj) {
        auto ros_traj = pyToCppMsg<trajectory_msgs::msg::JointTrajectory>(py_ros_traj);
        return fromJointTrajectory(ros_traj);
      },
      "py_ros_traj"_a,
      "Converts a trajectory_msgs::msg::JointTrajectory to a roboplan::JointTrajectory. Positions "
      "are copied as they are, which is only correct without continuous joints; use the overload "
      "that takes a scene otherwise.");

  m.def(
      "fromJointTrajectory",
      [](nb::handle py_ros_traj, const roboplan::Scene& py_scene) {
        auto ros_traj = pyToCppMsg<trajectory_msgs::msg::JointTrajectory>(py_ros_traj);
        return handle_expected(fromJointTrajectory(ros_traj, py_scene));
      },
      "py_ros_traj"_a, "py_scene"_a,
      "Converts a trajectory_msgs::msg::JointTrajectory to a roboplan::JointTrajectory. Continuous "
      "joints become [cos, sin] positions, and the values of mimic joints are ignored. Planar and "
      "floating joints are an error: use the overload that also takes a MultiDOFJointTrajectory.");

  m.def(
      "fromJointTrajectory",
      [](nb::handle py_ros_traj, nb::handle py_multi_dof_traj, const roboplan::Scene& py_scene) {
        auto ros_traj = pyToCppMsg<trajectory_msgs::msg::JointTrajectory>(py_ros_traj);
        auto multi_dof_traj =
            pyToCppMsg<trajectory_msgs::msg::MultiDOFJointTrajectory>(py_multi_dof_traj);
        return handle_expected(fromJointTrajectory(ros_traj, multi_dof_traj, py_scene));
      },
      "py_ros_traj"_a, "py_multi_dof_traj"_a, "py_scene"_a,
      "Converts a trajectory_msgs::msg::JointTrajectory with the single degree of freedom joints "
      "and a trajectory_msgs::msg::MultiDOFJointTrajectory with the planar and floating joints to "
      "one roboplan::JointTrajectory, with the joints of the JointTrajectory in their order, then "
      "those of the MultiDOFJointTrajectory in theirs.");

  m.def(
      "toTransformStamped",
      [](const roboplan::CartesianConfiguration& cartesian_config) {
        auto transform = toTransformStamped(cartesian_config);
        return cppToPyMsg(transform,
                          nb::module_::import_("geometry_msgs.msg").attr("TransformStamped"));
      },
      "cartesian_config"_a,
      "Converts a roboplan::CartesianConfiguration to a geometry_msgs::msg::TransformStamped.");

  m.def(
      "fromTransformStamped",
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
      "buildConversionMap",
      [](nb::handle py_scene, nb::handle py_joint_state) {
        const auto& scene = nb::cast<roboplan::Scene&>(py_scene);
        auto joint_state = pyToCppMsg<sensor_msgs::msg::JointState>(py_joint_state);
        return handle_expected(buildConversionMap(scene, joint_state));
      },
      "py_scene"_a, "py_joint_state"_a,
      "Constructs a JointState conversion map given a RoboPlan Scene and JointState message.");

  m.def(
      "toJointState",
      [](nb::handle py_config, nb::handle py_scene) {
        auto config = nb::cast<roboplan::JointConfiguration>(py_config);
        const auto& scene = nb::cast<roboplan::Scene&>(py_scene);
        auto result = handle_expected(toJointState(config, scene));
        return cppToPyMsg(result, nb::module_::import_("sensor_msgs.msg").attr("JointState"));
      },
      "py_config"_a, "py_scene"_a,
      "Converts a roboplan::JointConfiguration to a ROS 2 sensor_msgs::msg::JointState message.");

  m.def(
      "fromJointState",
      [](nb::handle py_joint_state, nb::handle py_scene,
         const JointStateConverterMap& conversion_map,
         const std::optional<Eigen::VectorXd>& q_reference) {
        auto joint_state = pyToCppMsg<sensor_msgs::msg::JointState>(py_joint_state);
        const auto& scene = nb::cast<roboplan::Scene&>(py_scene);
        return handle_expected(fromJointState(joint_state, scene, conversion_map, q_reference));
      },
      "py_joint_state"_a, "py_scene"_a, "conversion_map"_a, "q_reference"_a = nb::none(),
      "Converts a ROS 2 sensor_msgs::msg::JointState message to a roboplan::JointConfiguration. "
      "Joints that are not in the message keep their positions from q_reference, or from the "
      "scene's current state if it is not given.");
}
