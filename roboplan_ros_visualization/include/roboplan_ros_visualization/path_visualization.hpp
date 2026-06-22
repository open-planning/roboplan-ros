#pragma once

#include <optional>
#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace roboplan_ros_visualization {

/// @brief Builds a RViz line-list marker tracing a joint trajectory in Cartesian space.
/// @details Runs forward kinematics on each of the trajectory's positions (via
/// roboplan::computeFramePath) for every requested frame and connects the resulting frame origins
/// with line segments. Each frame name is traced independently, so the same marker can show the
/// Cartesian path of multiple end effectors at once. The trajectory's positions are group positions
/// (in the order of trajectory.joint_names), so they are embedded into a full configuration vector
/// seeded from the scene's current state before computing kinematics.
///
/// @param scene The scene used for forward kinematics.
/// @param trajectory The joint trajectory to trace. An empty trajectory yields an empty marker.
/// @param frame_names The frames (e.g., end effectors) whose origins are traced.
/// @param frame_id The frame_id written into the marker header.
/// @param ns The marker namespace.
/// @param color Optional line color. Defaults to opaque white when not provided.
/// @param line_width The rendered line width, in meters (Marker::scale.x).
/// @return A LINE_LIST marker containing the traced segments for every requested frame.
visualization_msgs::msg::Marker markerFromJointTrajectory(
    const roboplan::Scene& scene, const roboplan::JointTrajectory& trajectory,
    const std::vector<std::string>& frame_names, const std::string& frame_id = "world",
    const std::string& ns = "/roboplan_path",
    const std::optional<std_msgs::msg::ColorRGBA>& color = std::nullopt, double line_width = 0.01);

}  // namespace roboplan_ros_visualization
