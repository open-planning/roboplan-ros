#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
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

  /// IMPORTANT: We use a shared_ptr to manage ownership of the python Scene object between
  /// the python and C++ processes. This is critical! Otherwise problems with deconstruction
  /// and unexpected copying can cause all kinds of headaches.
  /// For more information: https://nanobind.readthedocs.io/en/latest/ownership.html
  nb::class_<roboplan_ros_cpp::RoboplanVisualizer>(
      m, "RoboplanVisualizer",
      "Tool to build RViz MarkerArray messages from a RoboPlan scene and joint configuration.")
      // Useful docs on custom constructors and why this is setup this way
      // https://nanobind.readthedocs.io/en/latest/porting.html#custom-constructors
      .def(
          "__init__",
          [](roboplan_ros_cpp::RoboplanVisualizer* self,
             std::shared_ptr<const roboplan::Scene> scene, const std::string& urdf_xml,
             const std::string& frame_id, const std::string& ns, nb::handle py_color) {
            std::optional<std_msgs::msg::ColorRGBA> color = std::nullopt;
            if (!py_color.is_none()) {
              color = pyToCppMsg<std_msgs::msg::ColorRGBA>(py_color);
            }
            new (self) roboplan_ros_cpp::RoboplanVisualizer(std::move(scene), urdf_xml, frame_id,
                                                            ns, color);
          },
          "scene"_a, "urdf_xml"_a, "frame_id"_a = "world", "ns"_a = "/roboplan",
          "color"_a = nb::none())
      .def(
          "markers_from_configuration",
          [MarkerArray](roboplan_ros_cpp::RoboplanVisualizer& self, const Eigen::VectorXd& q) {
            return cppToPyMsg(self.markers_from_configuration(q), MarkerArray);
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
