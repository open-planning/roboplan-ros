#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan_ros_cpp/roboplan_ros_cpp_bindings.hpp>
#include <roboplan_ros_cpp/type_conversions.hpp>

#include <roboplan_ros_visualization/roboplan_visualizer.hpp>

namespace nb = nanobind;
using namespace nb::literals;

/// @brief Python nanobind bindings for the roboplan_ros_visualization package.
/// @details Pulls in the roboplan and roboplan_ros_cpp bindings as necessary,
/// and installs the packages here into a "bindings" subpackage of the existing
/// python package.
NB_MODULE(bindings, m) {

  nb::class_<roboplan_ros_cpp::RoboplanVisualizer>(
      m, "RoboplanVisualizer",
      "Tool to build RViz MarkerArray messages from a RoboPlan scene and joint configuration.")
      .def(
          "__init__",
          [](roboplan_ros_cpp::RoboplanVisualizer* self, nb::handle py_scene,
             const std::string& urdf_xml, const std::vector<std::string>& package_paths,
             const std::string& frame_id, const std::string& ns, nb::handle py_color) {
            auto scene = nb::cast<roboplan::Scene>(py_scene);
            std::optional<std_msgs::msg::ColorRGBA> color = std::nullopt;
            if (!py_color.is_none()) {
              color = pyToCppMsg<std_msgs::msg::ColorRGBA>(py_color);
            }
            new (self) roboplan_ros_cpp::RoboplanVisualizer(scene, urdf_xml, package_paths,
                                                            frame_id, ns, color);
          },
          "scene"_a, "urdf_xml"_a, "package_paths"_a = std::vector<std::string>{},
          "frame_id"_a = "world", "ns"_a = "/roboplan", "color"_a = nb::none())
      .def("visualize_configuration",
           &roboplan_ros_cpp::RoboplanVisualizer::visualize_configuration, nb::arg("q"),
           "Compute marker array for the given joint configuration.")
      .def("clear_markers", &roboplan_ros_cpp::RoboplanVisualizer::clear_markers,
           "Return a MarkerArray that deletes all previously published markers.")
      .def("set_color", &roboplan_ros_cpp::RoboplanVisualizer::set_color, nb::arg("color"),
           "Set a color override for all geometry markers.")
      .def("clear_color", &roboplan_ros_cpp::RoboplanVisualizer::clear_color,
           "Remove the color override, reverting to per-geometry colors.");
}
