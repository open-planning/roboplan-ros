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

  // Pre-import Python messages to avoid repeated lookups
  nb::object MarkerArray = nb::module_::import_("visualization_msgs.msg").attr("MarkerArray");
  nb::object ColorRGBA = nb::module_::import_("std_msgs.msg").attr("ColorRGBA");

  /// IMPORTANT: RoboplanVisualizer stores a const reference to the Scene internally.
  /// We use nb::keep_alive<1, 2>() to ensure the Python Scene object outlives the
  /// visualizer, and nb::cast<roboplan::Scene&>() to obtain a reference rather than
  /// a copy. Without these, the Scene would be destroyed at the end of __init__ and
  /// the visualizer would hold a dangling reference. The same pattern must be applied
  /// to any binding that wraps a C++ class storing a reference to a Scene.
  nb::class_<roboplan_ros_cpp::RoboplanVisualizer>(
      m, "RoboplanVisualizer",
      "Tool to build RViz MarkerArray messages from a RoboPlan scene and joint configuration.")
      .def(
          "__init__",
          [](roboplan_ros_cpp::RoboplanVisualizer* self, nb::handle py_scene,
             const std::string& urdf_xml, const std::vector<std::string>& package_paths,
             const std::string& frame_id, const std::string& ns, nb::handle py_color) {
            // Ensure we cast the scene as a reference to avoid destruction issues from a copy.
            // Ownership between python and C++ is very tricky.
            auto& scene = nb::cast<roboplan::Scene&>(py_scene);
            std::optional<std_msgs::msg::ColorRGBA> color = std::nullopt;
            if (!py_color.is_none()) {
              color = pyToCppMsg<std_msgs::msg::ColorRGBA>(py_color);
            }
            new (self) roboplan_ros_cpp::RoboplanVisualizer(scene, urdf_xml, package_paths,
                                                            frame_id, ns, color);
          },
          // Ensure the scene is not destructed before the visualizer
          nb::keep_alive<1, 2>(), "scene"_a, "urdf_xml"_a,
          "package_paths"_a = std::vector<std::string>{}, "frame_id"_a = "world",
          "ns"_a = "/roboplan", "color"_a = nb::none())
      .def(
          "visualize_configuration",
          [MarkerArray](roboplan_ros_cpp::RoboplanVisualizer& self, const Eigen::VectorXd& q) {
            return cppToPyMsg(self.visualize_configuration(q), MarkerArray);
          },
          "q"_a, "Compute marker array for the given joint configuration.")
      .def(
          "clear_markers",
          [MarkerArray](roboplan_ros_cpp::RoboplanVisualizer& self) {
            return cppToPyMsg(self.clear_markers(), MarkerArray);
          },
          "Return a MarkerArray that deletes all previously published markers.")
      .def(
          "set_color",
          [](roboplan_ros_cpp::RoboplanVisualizer& self, nb::handle py_color) {
            self.set_color(pyToCppMsg<std_msgs::msg::ColorRGBA>(py_color));
          },
          "color"_a, "Set a color override for all geometry markers.")
      .def("clear_color", &roboplan_ros_cpp::RoboplanVisualizer::clear_color,
           "Remove the color override, reverting to per-geometry colors.");
}
