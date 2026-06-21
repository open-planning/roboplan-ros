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

namespace roboplan_ros_visualization {

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
  /// @param group_name Default joint group to render. When empty, the entire scene is rendered.
  ///                   Otherwise, only the geometries of to the named group's links are emitted.
  /// @param color Optional override color applied to every marker.
  RoboplanVisualizer(std::shared_ptr<const roboplan::Scene> scene, const std::string& urdf_xml,
                     const std::string& frame_id = "world", const std::string& ns = "/roboplan",
                     const std::string& group_name = "",
                     const std::optional<std_msgs::msg::ColorRGBA>& color = std::nullopt);

  /// @brief Compute visualization markers for the given joint configuration.
  /// @details Runs Pinocchio geometry placement updates and returns a MarkerArray that the caller
  /// can publish however they like. Only the geometry belonging to the currently selected group
  /// (see the constructor and set_group) is emitted, which is useful for cutting down on visual
  /// noise. The group's geometry selection is cached, so this is cheap to call repeatedly.
  /// @param q Joint positions (size must match the scene model's nq).
  /// @return MarkerArray with one marker per supported visual geometry in the selection.
  visualization_msgs::msg::MarkerArray markers_from_configuration(const Eigen::VectorXd& q) const;

  /// @brief Select the joint group whose links should be rendered.
  /// @details Recomputes and caches the geometry selection for the given group. Pass an empty
  /// string to render the entire scene. A group's links are derived from the scene's
  /// JointGroupInfo.
  /// @param group_name The joint group name, or an empty string for the whole scene.
  /// @throws std::runtime_error if the group name is not found in the scene.
  void set_group(const std::string& group_name);

  /// @brief Build a MarkerArray containing a single DELETEALL marker.
  static visualization_msgs::msg::MarkerArray clear_markers();

  /// @brief Override the color applied to every marker.
  void set_color(const std_msgs::msg::ColorRGBA& color);

  /// @brief Remove any color override so per-geometry colors are used.
  void clear_color();

private:
  /// @brief Determine which visual geometry objects belong to a joint group.
  /// @param group_name The joint group name, or an empty string for the whole scene.
  /// @return The indices into visual_model_.geometryObjects that should be rendered.
  /// For an empty group name, every geometry index in the model is returned.
  std::vector<std::size_t> geometry_indices_for_group(const std::string& group_name) const;

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

  /// @brief Currently selected joint group to render. Empty means render the entire scene.
  std::string group_name_;

  /// @brief Cached geometry indices for the selected group. Recomputed only when the group
  /// changes (construction or set_group), so rendering does not redo the selection each call.
  std::vector<std::size_t> geometry_indices_;

  /// @brief Optionally set to override colors of all markers before rendering
  std::optional<std_msgs::msg::ColorRGBA> color_;
};

}  // namespace roboplan_ros_visualization
