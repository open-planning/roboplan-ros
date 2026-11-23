#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <rclcpp/logging.hpp>

#include <roboplan_ros_cpp/type_conversions.hpp>
#include <roboplan_ros_visualization/geometry_visualization.hpp>
#include <roboplan_ros_visualization/roboplan_visualizer.hpp>

namespace roboplan_ros_visualization {

RoboplanVisualizer::RoboplanVisualizer(std::shared_ptr<const roboplan::Scene> scene,
                                       const std::string& urdf_xml, const std::string& frame_id,
                                       const std::string& ns, const std::string& group_name,
                                       const std::optional<std_msgs::msg::ColorRGBA>& color)
    : scene_(std::move(scene)), frame_id_(frame_id), ns_(ns), group_name_(group_name),
      color_(color) {
  // Build a visual GeometryModel from the URDF.
  // Scene only holds the collision model, so we need this for rendering.
  const auto& model = scene_->getModel();
  std::istringstream urdf_stream(urdf_xml);
  pinocchio::urdf::buildGeom(model, urdf_stream, pinocchio::VISUAL, visual_model_);

  // Resolve the group's geometry selection once up front. It is recomputed only when the group
  // changes via set_group, so rendering never redoes the selection on the hot path.
  geometry_indices_ = geometry_indices_for_group(group_name_);
}

visualization_msgs::msg::MarkerArray
RoboplanVisualizer::markers_from_configuration(const Eigen::VectorXd& q) const {
  visualization_msgs::msg::MarkerArray marker_array;

  const auto& model = scene_->getModel();

  pinocchio::Data data(model);
  pinocchio::GeometryData visual_data(visual_model_);
  pinocchio::updateGeometryPlacements(model, data, visual_model_, visual_data, q);

  // Only emit markers for the geometry objects that belong to the selected group.
  for (const std::size_t idx : geometry_indices_) {
    const auto& geom_obj = visual_model_.geometryObjects.at(idx);
    const pinocchio::SE3& placement = visual_data.oMg.at(idx);

    auto marker =
        create_geometry_marker(static_cast<int>(idx), frame_id_, ns_, geom_obj, placement, color_);
    if (marker) {
      marker_array.markers.push_back(std::move(*marker));
    } else {
      RCLCPP_WARN(rclcpp::get_logger("RoboplanVisualizer"), "%s", marker.error().c_str());
    }
  }

  return marker_array;
}

void RoboplanVisualizer::set_group(const std::string& group_name) {
  // Compute the new selection first so a bad group name leaves the visualizer unchanged.
  geometry_indices_ = geometry_indices_for_group(group_name);
  group_name_ = group_name;
}

std::vector<std::size_t>
RoboplanVisualizer::geometry_indices_for_group(const std::string& group_name) const {
  std::vector<std::size_t> indices;

  // An empty group name means "the whole scene", so render every geometry object.
  if (group_name.empty()) {
    indices.resize(visual_model_.geometryObjects.size());
    std::iota(indices.begin(), indices.end(), std::size_t{0});
    return indices;
  }

  const auto maybe_info = scene_->getJointGroupInfo(group_name);
  if (!maybe_info) {
    throw std::runtime_error("RoboplanVisualizer: " + maybe_info.error());
  }
  const auto& info = maybe_info.value();
  const auto& model = scene_->getModel();

  // A geometry object moves with the group when the joint it is attached to is one of the group's
  // joints or sits anywhere below one in the kinematic tree.
  std::vector<bool> moved_by_group(static_cast<std::size_t>(model.njoints), false);
  for (const auto joint_id : info.joint_indices) {
    for (const auto descendant : model.subtrees.at(joint_id)) {
      moved_by_group.at(descendant) = true;
    }
  }

  // Links explicitly listed in the group's SRDF definition are rendered as well.
  const std::unordered_set<std::string> link_set(info.link_names.begin(), info.link_names.end());

  for (std::size_t idx = 0; idx < visual_model_.geometryObjects.size(); ++idx) {
    const auto& geom_obj = visual_model_.geometryObjects.at(idx);
    const bool below_group_joint =
        geom_obj.parentJoint < moved_by_group.size() && moved_by_group[geom_obj.parentJoint];
    const bool listed_link = geom_obj.parentFrame < model.frames.size() &&
                             link_set.count(model.frames.at(geom_obj.parentFrame).name) > 0;
    if (below_group_joint || listed_link) {
      indices.push_back(idx);
    }
  }

  return indices;
}

visualization_msgs::msg::MarkerArray RoboplanVisualizer::clear_markers() {
  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker m;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  ma.markers.push_back(m);
  return ma;
}

void RoboplanVisualizer::set_color(const std_msgs::msg::ColorRGBA& color) { color_ = color; }

void RoboplanVisualizer::clear_color() { color_ = std::nullopt; }

}  // namespace roboplan_ros_visualization
