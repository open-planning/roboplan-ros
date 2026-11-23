#pragma once

#include <optional>
#include <string>

#include <pinocchio/multibody/geometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tl/expected.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace roboplan_ros_visualization {
/// @brief Create a visualization marker for a single geometry object.
/// @param marker_id Unique marker ID.
/// @param frame_id Frame id for the marker header.
/// @param ns Namespace for the generated marker.
/// @param geom_obj The Pinocchio geometry object to visualize.
/// @param placement The SE3 placement of the geometry in world frame.
/// @param color Optionally set to override colors of the marker before rendering.
/// @return A marker if the geometry type is supported, or a string describing
/// the error.
tl::expected<visualization_msgs::msg::Marker, std::string>
create_geometry_marker(int marker_id, const std::string& frame_id, const std::string& ns,
                       const pinocchio::GeometryObject& geom_obj, const pinocchio::SE3& placement,
                       std::optional<std_msgs::msg::ColorRGBA> color = std::nullopt);
}  // namespace roboplan_ros_visualization
