#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/geometry.hpp>

#include <roboplan/core/scene.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace roboplan_ros_cpp {

/// @brief Computes RViz visualization markers for a robot configuration.
///
/// Uses the Pinocchio model from a Scene and builds a visual GeometryModel
/// from the URDF to produce marker messages directly, no ROS node, TF, or
/// robot_state_publisher required.
class RoboplanVisualizer {
public:
  /// @brief Construct the visualizer.
  /// @param scene A fully configured RoboPlan Scene for model reference.
  /// @param urdf_xml URDF XML string (needed to build the visual geometry model,
  ///                 since Scene only holds the collision geometry model).
  /// @param frame_id The frame_id written into every marker header.
  /// @param ns Marker namespace prefix.
  /// @param color Optional override color applied to every marker.
  RoboplanVisualizer(std::shared_ptr<const roboplan::Scene> scene, const std::string& urdf_xml,
                     const std::string& frame_id = "world", const std::string& ns = "/roboplan",
                     const std::optional<std_msgs::msg::ColorRGBA>& color = std::nullopt);

  /// @brief Compute visualization markers for the given joint configuration.
  /// @details Runs Pinocchio geometry placement updates and returns a MarkerArray that
  /// the caller can publish however they like.
  /// @param q Joint positions (size must match the scene model's nq).
  /// @return MarkerArray with one marker per supported visual geometry.
  visualization_msgs::msg::MarkerArray markers_from_configuration(const Eigen::VectorXd& q) const;

  /// @brief Build a MarkerArray containing a single DELETEALL marker.
  static visualization_msgs::msg::MarkerArray clear_markers();

  /// @brief Override the color applied to every marker.
  void set_color(const std_msgs::msg::ColorRGBA& color);

  /// @brief Remove any color override so per-geometry colors are used.
  void clear_color();

private:
  /// @brief Create a visualization marker for a single geometry object.
  /// @param marker_id Unique marker ID.
  /// @param geom_obj The Pinocchio geometry object to visualize.
  /// @param placement The SE3 placement of the geometry in world frame.
  /// @return A marker if the geometry type is supported, std::nullopt otherwise.
  std::optional<visualization_msgs::msg::Marker>
  create_geometry_marker(int marker_id, const pinocchio::GeometryObject& geom_obj,
                         const pinocchio::SE3& placement) const;

  /// @brief Shared ptr to avoid ownership issues between C++ and Python
  std::shared_ptr<const roboplan::Scene> scene_;

  /// @brief Visual geometry model built from URDF
  pinocchio::GeometryModel visual_model_;

  /// @brief Frame id for the marker headers
  std::string frame_id_;

  /// @brief Namespace for the generated markers
  std::string ns_;

  /// @brief Optionally set to override colors of all markers before rendering
  std::optional<std_msgs::msg::ColorRGBA> color_;
};

}  // namespace roboplan_ros_cpp
