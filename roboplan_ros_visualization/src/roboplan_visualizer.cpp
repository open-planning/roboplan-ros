#include "roboplan_ros_visualization/roboplan_visualizer.hpp"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_cpp {

RoboplanVisualizer::RoboplanVisualizer(const roboplan::Scene& scene, const std::string& urdf_xml,
                                       const std::vector<std::string>& package_paths,
                                       const std::string& frame_id, const std::string& ns,
                                       const std::optional<std_msgs::msg::ColorRGBA>& color)
    : scene_(scene), frame_id_(frame_id), ns_(ns), color_(color) {
  // Build a visual GeometryModel from the URDF.
  // Scene only holds the collision model, so we need this for rendering.
  const auto& model = scene_.getModel();
  std::istringstream urdf_stream(urdf_xml);
  pinocchio::urdf::buildGeom(model, urdf_stream, pinocchio::VISUAL, visual_model_, package_paths);
}

visualization_msgs::msg::MarkerArray
RoboplanVisualizer::visualize_configuration(const Eigen::VectorXd& q) const {
  visualization_msgs::msg::MarkerArray marker_array;

  const auto& model = scene_.getModel();

  pinocchio::Data data(model);
  pinocchio::GeometryData visual_data(visual_model_);
  pinocchio::updateGeometryPlacements(model, data, visual_model_, visual_data, q);

  for (std::size_t idx = 0; idx < visual_model_.geometryObjects.size(); ++idx) {
    const auto& geom_obj = visual_model_.geometryObjects[idx];
    const pinocchio::SE3& placement = visual_data.oMg[idx];

    auto marker = createGeometryMarker(static_cast<int>(idx), geom_obj, placement);
    if (marker) {
      marker_array.markers.push_back(std::move(*marker));
    }
  }

  return marker_array;
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

std::optional<visualization_msgs::msg::Marker>
RoboplanVisualizer::createGeometryMarker(int marker_id, const pinocchio::GeometryObject& geom_obj,
                                         const pinocchio::SE3& placement) const {

  // The caller can change as needed
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.ns = ns_ + "/" + geom_obj.name;
  marker.id = marker_id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = roboplan_ros_cpp::se3ToPose(placement.toHomogeneousMatrix());

  // Dispatch on hpp-fcl geometry type.
  const auto* geom = geom_obj.geometry.get();

  if (const auto* cyl = dynamic_cast<const hpp::fcl::Cylinder*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = marker.scale.y = cyl->radius * 2.0;
    marker.scale.z = cyl->halfLength * 2.0;

  } else if (const auto* cap = dynamic_cast<const hpp::fcl::Capsule*>(geom)) {
    // Capsules are just be approximated as cylinders for now...
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = marker.scale.y = cap->radius * 2.0;
    marker.scale.z = cap->halfLength * 2.0;

  } else if (const auto* box = dynamic_cast<const hpp::fcl::Box*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = box->halfSide[0] * 2.0;
    marker.scale.y = box->halfSide[1] * 2.0;
    marker.scale.z = box->halfSide[2] * 2.0;

  } else if (const auto* sph = dynamic_cast<const hpp::fcl::Sphere*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    const double d = sph->radius * 2.0;
    marker.scale.x = marker.scale.y = marker.scale.z = d;

  } else if (dynamic_cast<const hpp::fcl::Cone*>(geom)) {
    // No cone markers are available so...
    // TODO: Make a cone?
    return std::nullopt;

  } else if (dynamic_cast<const hpp::fcl::ConvexBase*>(geom) ||
             dynamic_cast<const hpp::fcl::BVHModelBase*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

    const std::string& mesh_path = geom_obj.meshPath;
    if (mesh_path.empty()) {
      return std::nullopt;
    }

    // Ensure a proper URI scheme so RViz doesn't complain.
    if (mesh_path.rfind("package://", 0) == 0 || mesh_path.rfind("file://", 0) == 0) {
      marker.mesh_resource = mesh_path;
    } else {
      marker.mesh_resource = "file://" + mesh_path;
    }

    marker.scale.x = geom_obj.meshScale[0];
    marker.scale.y = geom_obj.meshScale[1];
    marker.scale.z = geom_obj.meshScale[2];

  } else {
    // TODO: What should we do with these?
    return std::nullopt;
  }

  // Use embedded materials for meshes, if available
  if (marker.type == visualization_msgs::msg::Marker::MESH_RESOURCE) {
    marker.mesh_use_embedded_materials = true;
  }

  // Apply the geometry object's own colour (meshColor is Eigen::Vector4d)
  if (geom_obj.meshColor.size() >= 4) {
    marker.color.r = static_cast<float>(geom_obj.meshColor[0]);
    marker.color.g = static_cast<float>(geom_obj.meshColor[1]);
    marker.color.b = static_cast<float>(geom_obj.meshColor[2]);
    marker.color.a = static_cast<float>(geom_obj.meshColor[3]);
  }

  // Apply the supplied override, if present
  if (color_) {
    marker.color = *color_;
  }

  return marker;
}

}  // namespace roboplan_ros_cpp
