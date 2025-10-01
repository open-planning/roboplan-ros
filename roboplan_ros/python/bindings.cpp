#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "roboplan_ros/type_conversions.hpp"
#include <roboplan/core/types.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace nb = nanobind;

NB_MODULE(roboplan_ros_py, m) {
    m.doc() = "Python bindings for the ROS 2 wrappers of the roboplan library";

    // TODO: This shouldn't be necessary but I haven't got the bindings building yet...
    nb::class_<roboplan::JointTrajectory>(m, "JointTrajectory")
        .def(nb::init<>())
        .def_rw("joint_names", &roboplan::JointTrajectory::joint_names)
        .def_rw("times", &roboplan::JointTrajectory::times)
        .def_rw("positions", &roboplan::JointTrajectory::positions)
        .def_rw("velocities", &roboplan::JointTrajectory::velocities)
        .def_rw("accelerations", &roboplan::JointTrajectory::accelerations);

    m.def("toJointTrajectory", &roboplan_ros::toJointTrajectory,
          nb::arg("roboplan_trajectory"),
          "Converts a roboplan::JointTrajectory to a ROS 2 JointTrajectory message");

    m.def("fromJointTrajectory", &roboplan_ros::fromJointTrajectory,
          nb::arg("ros_trajectory"),
          "Converts a ROS 2 JointTrajectory message to a roboplan::JointTrajectory");
}
