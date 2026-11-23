#include <roboplan_ros_visualization/geometry_visualization.hpp>

// hppfcl was renamed to coal. Prefer the coal headers whenever they are available,
// since modern hpp-fcl releases declare `namespace coal { }` as a real namespace
// (with `hpp::fcl` as the backward-compatibility alias), which conflicts with
// declaring `namespace coal = hpp::fcl;` ourselves.
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
#include <coal/BVH/BVH_model.h>
#include <coal/shape/convex.h>
#include <coal/shape/geometric_shapes.h>
#else
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/shape/convex.h>
#include <hpp/fcl/shape/geometric_shapes.h>
namespace coal = hpp::fcl;
#endif

#include <roboplan_ros_cpp/type_conversions.hpp>

namespace roboplan_ros_visualization {

tl::expected<visualization_msgs::msg::Marker, std::string>
create_geometry_marker(int marker_id, const std::string& frame_id, const std::string& ns,
                       const pinocchio::GeometryObject& geom_obj, const pinocchio::SE3& placement,
                       std::optional<std_msgs::msg::ColorRGBA> color) {

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns + "/" + geom_obj.name;
  marker.id = marker_id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = roboplan_ros_cpp::se3ToPose(placement.toHomogeneousMatrix());

  // Dispatch on hpp-fcl geometry type.
  const auto* geom = geom_obj.geometry.get();

  if (const auto* cyl = dynamic_cast<const coal::Cylinder*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = marker.scale.y = cyl->radius * 2.0;
    marker.scale.z = cyl->halfLength * 2.0;

  } else if (const auto* cap = dynamic_cast<const coal::Capsule*>(geom)) {
    // Capsules are just be approximated as cylinders for now...
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = marker.scale.y = cap->radius * 2.0;
    marker.scale.z = cap->halfLength * 2.0;

  } else if (const auto* box = dynamic_cast<const coal::Box*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = box->halfSide[0] * 2.0;
    marker.scale.y = box->halfSide[1] * 2.0;
    marker.scale.z = box->halfSide[2] * 2.0;

  } else if (const auto* sph = dynamic_cast<const coal::Sphere*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    const double d = sph->radius * 2.0;
    marker.scale.x = marker.scale.y = marker.scale.z = d;

  } else if (dynamic_cast<const coal::Cone*>(geom)) {
    // No cone markers are available so...
    // TODO: Make a cone?
    return tl::make_unexpected("Cone geometry unsupported for " + geom_obj.name);
  } else if (dynamic_cast<const coal::ConvexBase*>(geom) ||
             dynamic_cast<const coal::BVHModelBase*>(geom)) {
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

    const std::string& mesh_path = geom_obj.meshPath;
    if (mesh_path.empty()) {
      return tl::make_unexpected("geom_obj.meshPath is empty for " + geom_obj.name);
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
    return tl::make_unexpected("Unsupported geometry type for " + geom_obj.name);
  }

  // Use embedded materials for meshes, if available
  if (marker.type == visualization_msgs::msg::Marker::MESH_RESOURCE) {
    marker.mesh_use_embedded_materials = true;
  }

  // Apply the geometry object's own color
  if (geom_obj.meshColor.size() >= 4) {
    marker.color.r = static_cast<float>(geom_obj.meshColor[0]);
    marker.color.g = static_cast<float>(geom_obj.meshColor[1]);
    marker.color.b = static_cast<float>(geom_obj.meshColor[2]);
    marker.color.a = static_cast<float>(geom_obj.meshColor[3]);
  }

  // Apply the supplied override, if present
  if (color) {
    marker.color = *color;
  }

  return marker;
}

}  // namespace roboplan_ros_visualization
