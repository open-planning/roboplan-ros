#include <roboplan_ros_cpp/type_conversions.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <unordered_set>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace roboplan_ros_cpp {

namespace {

/// @brief How a trajectory joint is represented in RoboPlan and in ROS.
enum class TrajectoryJointKind {
  /// @brief A revolute or prismatic joint: one value in both.
  SINGLE,
  /// @brief [cos(theta), sin(theta)] in RoboPlan, theta in a JointTrajectory.
  CONTINUOUS,
  /// @brief No values of its own in RoboPlan, which derives them from the joint it mimics.
  MIMIC,
  /// @brief [x, y, cos(theta), sin(theta)] in RoboPlan, a transform in a MultiDOFJointTrajectory.
  PLANAR,
  /// @brief [x, y, z, qx, qy, qz, qw] in RoboPlan, a transform in a MultiDOFJointTrajectory.
  FLOATING,
};

/// @brief A joint of a trajectory, with the number of values it has in RoboPlan.
struct TrajectoryJoint {
  std::string name;
  TrajectoryJointKind kind;

  /// @brief Number of positions in RoboPlan.
  size_t nq;

  /// @brief Number of velocities (and accelerations) in RoboPlan.
  size_t nv;

  bool isMultiDof() const {
    return kind == TrajectoryJointKind::PLANAR || kind == TrajectoryJointKind::FLOATING;
  }
};

/// @brief Looks up how each joint of a trajectory is represented.
/// @return The joints in the order of joint_names, or an error if one is unknown.
tl::expected<std::vector<TrajectoryJoint>, std::string>
trajectoryJoints(const roboplan::Scene& scene, const std::vector<std::string>& joint_names) {
  std::vector<TrajectoryJoint> joints;
  joints.reserve(joint_names.size());
  for (const auto& joint_name : joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (joint_info->mimic_info) {
      joints.push_back({joint_name, TrajectoryJointKind::MIMIC, 0, 0});
      continue;
    }
    switch (joint_info->type) {
    case roboplan::JointType::REVOLUTE:
    case roboplan::JointType::PRISMATIC:
      joints.push_back({joint_name, TrajectoryJointKind::SINGLE, 1, 1});
      break;
    case roboplan::JointType::CONTINUOUS:
      joints.push_back({joint_name, TrajectoryJointKind::CONTINUOUS, 2, 1});
      break;
    case roboplan::JointType::PLANAR:
      joints.push_back({joint_name, TrajectoryJointKind::PLANAR, 4, 3});
      break;
    case roboplan::JointType::FLOATING:
      joints.push_back({joint_name, TrajectoryJointKind::FLOATING, 7, 6});
      break;
    default:
      return tl::make_unexpected("Joint '" + joint_name + "' has an unknown type");
    }
  }
  return joints;
}

/// @brief Total number of RoboPlan positions and velocities of some trajectory joints.
struct TrajectorySize {
  size_t nq = 0;
  size_t nv = 0;
};

TrajectorySize trajectorySize(const std::vector<TrajectoryJoint>& joints) {
  TrajectorySize size;
  for (const auto& joint : joints) {
    size.nq += joint.nq;
    size.nv += joint.nv;
  }
  return size;
}

/// @brief Checks that a point's values have the expected size, or are empty if they are optional.
tl::expected<void, std::string> checkPointSize(const std::string& what, size_t point_index,
                                               size_t size, size_t expected, bool optional) {
  if (size == expected || (optional && size == 0)) {
    return {};
  }
  return tl::make_unexpected("Failed to convert the trajectory: Point " +
                             std::to_string(point_index) + " has " + std::to_string(size) + " " +
                             what + ", expected " + std::to_string(expected));
}

/// @brief Checks the sizes of the values of point i of a RoboPlan trajectory.
tl::expected<void, std::string> checkRoboPlanPoint(const roboplan::JointTrajectory& trajectory,
                                                   size_t i, const TrajectorySize& size) {
  if (i < trajectory.positions.size()) {
    if (auto ok = checkPointSize("positions", i, trajectory.positions[i].size(), size.nq, false);
        !ok) {
      return ok;
    }
  }
  if (i < trajectory.velocities.size()) {
    if (auto ok = checkPointSize("velocities", i, trajectory.velocities[i].size(), size.nv, true);
        !ok) {
      return ok;
    }
  }
  if (i < trajectory.accelerations.size()) {
    return checkPointSize("accelerations", i, trajectory.accelerations[i].size(), size.nv, true);
  }
  return {};
}

/// @brief Returns point i's values, or nullptr if the trajectory has none for it.
const Eigen::VectorXd* pointValues(const std::vector<Eigen::VectorXd>& values, size_t i) {
  return i < values.size() && values[i].size() > 0 ? &values[i] : nullptr;
}

/// @brief Returns angle plus the multiple of 2 pi that brings it closest to previous, if any.
double unwrapAngle(double angle, const std::optional<double>& previous) {
  if (!previous) {
    return angle;
  }
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  return angle + kTwoPi * std::round((*previous - angle) / kTwoPi);
}

/// @brief Converts the RoboPlan velocities (or accelerations) of a planar or floating joint to a
/// twist, both in the joint's own frame.
geometry_msgs::msg::Twist toTwist(TrajectoryJointKind kind, const Eigen::VectorXd& values,
                                  size_t v_idx) {
  geometry_msgs::msg::Twist twist;
  if (kind == TrajectoryJointKind::PLANAR) {
    twist.linear.x = values(v_idx);
    twist.linear.y = values(v_idx + 1);
    twist.angular.z = values(v_idx + 2);
  } else {
    twist.linear.x = values(v_idx);
    twist.linear.y = values(v_idx + 1);
    twist.linear.z = values(v_idx + 2);
    twist.angular.x = values(v_idx + 3);
    twist.angular.y = values(v_idx + 4);
    twist.angular.z = values(v_idx + 5);
  }
  return twist;
}

/// @brief Inverse of toTwist: writes a twist's values for a planar or floating joint.
void fromTwist(TrajectoryJointKind kind, const geometry_msgs::msg::Twist& twist,
               Eigen::VectorXd& values, size_t v_idx) {
  if (kind == TrajectoryJointKind::PLANAR) {
    values.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
  } else {
    values.segment<6>(v_idx) << twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x,
        twist.angular.y, twist.angular.z;
  }
}

/// @brief Converts a ROS JointTrajectory and MultiDOFJointTrajectory pair to one RoboPlan
/// trajectory. The joints are in the order of the model if scene_order is true, otherwise those
/// of ros_trajectory come first in their order, then those of multi_dof_trajectory.
tl::expected<roboplan::JointTrajectory, std::string>
fromRosTrajectories(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene, bool scene_order) {
  const auto error = [](const std::string& message) {
    return tl::make_unexpected("Failed to convert the trajectory: " + message);
  };

  // Each joint, with whether it comes from the MultiDOFJointTrajectory and its index there.
  struct Source {
    TrajectoryJoint joint;
    bool multi_dof;
    size_t index;
  };
  std::vector<Source> sources;
  for (const bool multi_dof : {false, true}) {
    const auto& names = multi_dof ? multi_dof_trajectory.joint_names : ros_trajectory.joint_names;
    const auto joints = trajectoryJoints(scene, names);
    if (!joints) {
      return error(joints.error());
    }
    for (size_t j = 0; j < joints->size(); ++j) {
      const auto& joint = joints->at(j);
      if (joint.isMultiDof() && !multi_dof) {
        return error("Joint '" + joint.name +
                     "' has several degrees of freedom: a JointTrajectory cannot represent it, "
                     "use a MultiDOFJointTrajectory");
      }
      if (!joint.isMultiDof() && multi_dof) {
        return error("Joint '" + joint.name +
                     "' has a single degree of freedom: it belongs in the JointTrajectory");
      }
      sources.push_back({joint, multi_dof, j});
    }
  }
  std::unordered_set<std::string> seen;
  for (const auto& source : sources) {
    if (!seen.insert(source.joint.name).second) {
      return error("Joint '" + source.joint.name + "' appears twice");
    }
  }
  if (scene_order) {
    const auto& model = scene.getModel();
    std::stable_sort(sources.begin(), sources.end(), [&model](const auto& a, const auto& b) {
      return model.getJointId(a.joint.name) < model.getJointId(b.joint.name);
    });
  }

  // A message without joints only matters if the other has none either.
  const bool use_multi_dof = !multi_dof_trajectory.joint_names.empty();
  const bool use_single = !ros_trajectory.joint_names.empty() || !use_multi_dof;
  const auto num_points =
      use_single ? ros_trajectory.points.size() : multi_dof_trajectory.points.size();
  if (use_single && use_multi_dof && multi_dof_trajectory.points.size() != num_points) {
    return error("The JointTrajectory has " + std::to_string(num_points) +
                 " points, the MultiDOFJointTrajectory " +
                 std::to_string(multi_dof_trajectory.points.size()));
  }

  TrajectorySize size;
  for (const auto& source : sources) {
    size.nq += source.joint.nq;
    size.nv += source.joint.nv;
  }
  const auto num_single = ros_trajectory.joint_names.size();
  const auto num_multi_dof = multi_dof_trajectory.joint_names.size();

  roboplan::JointTrajectory joint_traj;
  for (const auto& source : sources) {
    joint_traj.joint_names.push_back(source.joint.name);
  }
  joint_traj.times.reserve(num_points);
  joint_traj.positions.reserve(num_points);
  joint_traj.velocities.reserve(num_points);
  joint_traj.accelerations.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const auto* point = use_single ? &ros_trajectory.points[i] : nullptr;
    const auto* multi_dof_point = use_multi_dof ? &multi_dof_trajectory.points[i] : nullptr;
    if (point && multi_dof_point && point->time_from_start != multi_dof_point->time_from_start) {
      return error("Point " + std::to_string(i) +
                   " has different times in the JointTrajectory and the MultiDOFJointTrajectory");
    }
    if (point) {
      if (auto ok = checkPointSize("positions", i, point->positions.size(), num_single, false);
          !ok) {
        return tl::make_unexpected(ok.error());
      }
      if (auto ok = checkPointSize("velocities", i, point->velocities.size(), num_single, true);
          !ok) {
        return tl::make_unexpected(ok.error());
      }
      if (auto ok =
              checkPointSize("accelerations", i, point->accelerations.size(), num_single, true);
          !ok) {
        return tl::make_unexpected(ok.error());
      }
    }
    if (multi_dof_point) {
      if (auto ok = checkPointSize("transforms", i, multi_dof_point->transforms.size(),
                                   num_multi_dof, false);
          !ok) {
        return tl::make_unexpected(ok.error());
      }
      if (auto ok = checkPointSize("velocity twists", i, multi_dof_point->velocities.size(),
                                   num_multi_dof, true);
          !ok) {
        return tl::make_unexpected(ok.error());
      }
      if (auto ok = checkPointSize("acceleration twists", i, multi_dof_point->accelerations.size(),
                                   num_multi_dof, true);
          !ok) {
        return tl::make_unexpected(ok.error());
      }
    }

    // Velocities and accelerations must be given for all the joints or for none.
    const auto all_or_none = [&](bool single_has, bool multi_dof_has,
                                 const std::string& what) -> tl::expected<bool, std::string> {
      if (point && multi_dof_point && single_has != multi_dof_has) {
        return error("Point " + std::to_string(i) + " has " + what + " for some joints only");
      }
      return point ? single_has : multi_dof_has;
    };
    const auto has_velocities =
        all_or_none(point && !point->velocities.empty(),
                    multi_dof_point && !multi_dof_point->velocities.empty(), "velocities");
    if (!has_velocities) {
      return tl::make_unexpected(has_velocities.error());
    }
    const auto has_accelerations =
        all_or_none(point && !point->accelerations.empty(),
                    multi_dof_point && !multi_dof_point->accelerations.empty(), "accelerations");
    if (!has_accelerations) {
      return tl::make_unexpected(has_accelerations.error());
    }

    Eigen::VectorXd positions(size.nq);
    Eigen::VectorXd velocities(*has_velocities ? size.nv : 0);
    Eigen::VectorXd accelerations(*has_accelerations ? size.nv : 0);
    size_t q_idx = 0;
    size_t v_idx = 0;
    for (const auto& source : sources) {
      const auto j = source.index;
      switch (source.joint.kind) {
      case TrajectoryJointKind::MIMIC:
        // Mimic joints have no values of their own: RoboPlan derives them.
        break;
      case TrajectoryJointKind::SINGLE:
      case TrajectoryJointKind::CONTINUOUS:
        if (source.joint.kind == TrajectoryJointKind::CONTINUOUS) {
          positions(q_idx) = std::cos(point->positions[j]);
          positions(q_idx + 1) = std::sin(point->positions[j]);
        } else {
          positions(q_idx) = point->positions[j];
        }
        if (velocities.size() > 0) {
          velocities(v_idx) = point->velocities[j];
        }
        if (accelerations.size() > 0) {
          accelerations(v_idx) = point->accelerations[j];
        }
        break;
      case TrajectoryJointKind::PLANAR:
      case TrajectoryJointKind::FLOATING: {
        const auto& transform = multi_dof_point->transforms[j];
        Eigen::Quaterniond rotation;
        tf2::fromMsg(transform.rotation, rotation);
        if (rotation.norm() < 1e-9) {
          return error("Point " + std::to_string(i) + " has a zero quaternion for joint '" +
                       source.joint.name + "'");
        }
        rotation.normalize();
        if (source.joint.kind == TrajectoryJointKind::PLANAR) {
          // Only the rotation about z: the direction of the x axis in the xy plane.
          const Eigen::Matrix3d matrix = rotation.toRotationMatrix();
          const double norm = std::hypot(matrix(0, 0), matrix(1, 0));
          if (norm < 1e-9) {
            return error("Point " + std::to_string(i) + " has no rotation about z for joint '" +
                         source.joint.name + "'");
          }
          positions.segment<4>(q_idx) << transform.translation.x, transform.translation.y,
              matrix(0, 0) / norm, matrix(1, 0) / norm;
        } else {
          positions.segment<7>(q_idx) << transform.translation.x, transform.translation.y,
              transform.translation.z, rotation.x(), rotation.y(), rotation.z(), rotation.w();
        }
        if (velocities.size() > 0) {
          fromTwist(source.joint.kind, multi_dof_point->velocities[j], velocities, v_idx);
        }
        if (accelerations.size() > 0) {
          fromTwist(source.joint.kind, multi_dof_point->accelerations[j], accelerations, v_idx);
        }
        break;
      }
      }
      q_idx += source.joint.nq;
      v_idx += source.joint.nv;
    }
    joint_traj.times.push_back(
        fromDuration(point ? point->time_from_start : multi_dof_point->time_from_start));
    joint_traj.positions.push_back(positions);
    joint_traj.velocities.push_back(velocities);
    joint_traj.accelerations.push_back(accelerations);
  }
  return joint_traj;
}

}  // namespace

tl::expected<JointStateConverterMap, std::string>
buildConversionMap(const roboplan::Scene& scene, const sensor_msgs::msg::JointState& joint_state) {
  // Setup a mapping joint names to their index in the joint state
  const auto& ros_joint_names = joint_state.name;
  std::unordered_map<std::string, size_t> ros_map;
  ros_map.reserve(ros_joint_names.size());
  for (size_t i = 0; i < ros_joint_names.size(); ++i) {
    ros_map[ros_joint_names[i]] = i;
  }

  // Pull out actuated joint state sizes
  const auto& model = scene.getModel();
  JointStateConverterMap conversion_map;
  conversion_map.nq = model.nq;
  conversion_map.nv = model.nv;
  conversion_map.mappings.reserve(joint_state.name.size());

  // For each actuated joint name, determine type and its location in the Scene's configuration,
  // and put it into the mapping.
  for (const auto& joint_name : joint_state.name) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected("Failed to get joint info for: " + joint_name);
    }
    const auto& info = joint_info.value();

    // TODO: This might not actually be worth failing for
    auto it = ros_map.find(joint_name);
    if (it == ros_map.end()) {
      return tl::make_unexpected("Actuated joint: " + joint_name + ", not in the ROS message!");
    }

    // Grab the joint info and store the relevant information
    const auto joint_id = model.getJointId(joint_name);
    const auto q_idx = model.idx_qs.at(joint_id);
    const auto v_idx = model.idx_vs.at(joint_id);
    JointStateConverterMap::JointMapping mapping{.joint_name = joint_name,
                                                 .ros_index = it->second,
                                                 .q_start = static_cast<size_t>(q_idx),
                                                 .v_start = static_cast<size_t>(v_idx),
                                                 .type = info.type};
    conversion_map.mappings.push_back(mapping);
  }

  return conversion_map;
}

tl::expected<sensor_msgs::msg::JointState, std::string>
toJointState(const roboplan::JointConfiguration& config, const roboplan::Scene& scene) {
  const auto& model = scene.getModel();

  sensor_msgs::msg::JointState msg;
  msg.name.reserve(scene.getJointNames().size());
  msg.position.reserve(scene.getJointNames().size());
  msg.velocity.reserve(scene.getJointNames().size());
  msg.effort.reserve(scene.getJointNames().size());

  for (const auto& joint_name : scene.getJointNames()) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected("Failed to get joint info: " + joint_name);
    }
    const auto& info = joint_info.value();

    const auto joint_id = model.getJointId(joint_name);
    const auto q_idx = model.idx_qs.at(joint_id);
    const auto v_idx = model.idx_vs.at(joint_id);

    msg.name.push_back(joint_name);
    if (info.type == roboplan::JointType::CONTINUOUS) {
      const auto theta = std::atan2(config.positions(q_idx + 1), config.positions(q_idx));
      msg.position.push_back(theta);
    } else {
      msg.position.push_back(config.positions(q_idx));
    }

    msg.velocity.push_back(v_idx < config.velocities.size() ? config.velocities(v_idx) : 0.0);
    msg.effort.push_back(v_idx < config.accelerations.size() ? config.accelerations(v_idx) : 0.0);
  }

  return msg;
}

tl::expected<roboplan::JointConfiguration, std::string>
fromJointState(const sensor_msgs::msg::JointState& joint_state, const roboplan::Scene& scene,
               const JointStateConverterMap& conversion_map,
               const std::optional<Eigen::VectorXd>& q_reference) {
  if (q_reference && static_cast<size_t>(q_reference->size()) != conversion_map.nq) {
    return tl::make_unexpected("q_reference has " + std::to_string(q_reference->size()) +
                               " positions, the scene has " + std::to_string(conversion_map.nq));
  }

  roboplan::JointConfiguration config;
  config.joint_names = scene.getJointNames();
  // Joints that are not in the message keep their reference positions.
  config.positions = q_reference ? *q_reference : scene.getCurrentJointPositions();
  config.velocities = Eigen::VectorXd::Zero(conversion_map.nv);
  config.accelerations = Eigen::VectorXd::Zero(conversion_map.nv);

  // Process actuated joints using pre-computed indices
  for (const auto& mapping : conversion_map.mappings) {
    const size_t ros_idx = mapping.ros_index;

    if (ros_idx >= joint_state.position.size()) {
      return tl::make_unexpected("Missing position data for joint '" + mapping.joint_name + "'");
    }

    if (mapping.type == roboplan::JointType::CONTINUOUS) {
      const double theta = joint_state.position.at(ros_idx);
      config.positions(mapping.q_start) = std::cos(theta);
      config.positions(mapping.q_start + 1) = std::sin(theta);
      if (ros_idx < joint_state.velocity.size()) {
        config.velocities(mapping.v_start) = joint_state.velocity.at(ros_idx);
      }
      if (ros_idx < joint_state.effort.size()) {
        config.accelerations(mapping.v_start) = joint_state.effort.at(ros_idx);
      }
    } else {
      config.positions(mapping.q_start) = joint_state.position.at(ros_idx);

      if (ros_idx < joint_state.velocity.size()) {
        config.velocities(mapping.v_start) = joint_state.velocity.at(ros_idx);
      }
      if (ros_idx < joint_state.effort.size()) {
        config.accelerations(mapping.v_start) = joint_state.effort.at(ros_idx);
      }
    }
  }

  return config;
}

trajectory_msgs::msg::JointTrajectory
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory) {

  trajectory_msgs::msg::JointTrajectory ros_traj;
  ros_traj.joint_names = roboplan_trajectory.joint_names;

  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < roboplan_trajectory.times.size(); ++i) {
    auto& point = ros_traj.points[i];
    double time_sec = roboplan_trajectory.times[i];
    point.time_from_start = toDuration(time_sec);

    if (i < roboplan_trajectory.positions.size()) {
      point.positions.resize(roboplan_trajectory.positions[i].size());
      for (int j = 0; j < roboplan_trajectory.positions[i].size(); ++j) {
        point.positions[j] = roboplan_trajectory.positions[i][j];
      }
    }

    if (i < roboplan_trajectory.velocities.size()) {
      point.velocities.resize(roboplan_trajectory.velocities[i].size());
      for (int j = 0; j < roboplan_trajectory.velocities[i].size(); ++j) {
        point.velocities[j] = roboplan_trajectory.velocities[i][j];
      }
    }

    if (i < roboplan_trajectory.accelerations.size()) {
      point.accelerations.resize(roboplan_trajectory.accelerations[i].size());
      for (int j = 0; j < roboplan_trajectory.accelerations[i].size(); ++j) {
        point.accelerations[j] = roboplan_trajectory.accelerations[i][j];
      }
    }
  }

  return ros_traj;
}

roboplan::JointTrajectory
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory) {

  roboplan::JointTrajectory joint_traj;

  joint_traj.joint_names = ros_trajectory.joint_names;

  joint_traj.times.reserve(ros_trajectory.points.size());
  joint_traj.positions.reserve(ros_trajectory.points.size());
  joint_traj.velocities.reserve(ros_trajectory.points.size());
  joint_traj.accelerations.reserve(ros_trajectory.points.size());

  for (const auto& point : ros_trajectory.points) {
    joint_traj.times.push_back(fromDuration(point.time_from_start));

    Eigen::VectorXd positions =
        Eigen::VectorXd::Map(point.positions.data(), point.positions.size());
    joint_traj.positions.push_back(positions);

    Eigen::VectorXd velocities =
        Eigen::VectorXd::Map(point.velocities.data(), point.velocities.size());
    joint_traj.velocities.push_back(velocities);

    Eigen::VectorXd accelerations =
        Eigen::VectorXd::Map(point.accelerations.data(), point.accelerations.size());
    joint_traj.accelerations.push_back(accelerations);
  }

  return joint_traj;
}

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene) {
  const auto joints = trajectoryJoints(scene, roboplan_trajectory.joint_names);
  if (!joints) {
    return tl::make_unexpected("Failed to convert the trajectory: " + joints.error());
  }
  const auto size = trajectorySize(joints.value());

  // Mimic joints have no values of their own, planar and floating joints go in a
  // MultiDOFJointTrajectory.
  const auto in_message = [](const TrajectoryJoint& joint) {
    return joint.kind == TrajectoryJointKind::SINGLE ||
           joint.kind == TrajectoryJointKind::CONTINUOUS;
  };
  trajectory_msgs::msg::JointTrajectory ros_traj;
  for (const auto& joint : joints.value()) {
    if (in_message(joint)) {
      ros_traj.joint_names.push_back(joint.name);
    }
  }

  std::vector<std::optional<double>> previous_angles(joints->size());
  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < ros_traj.points.size(); ++i) {
    if (auto ok = checkRoboPlanPoint(roboplan_trajectory, i, size); !ok) {
      return tl::make_unexpected(ok.error());
    }
    auto& point = ros_traj.points[i];
    point.time_from_start = toDuration(roboplan_trajectory.times[i]);
    const auto* positions = pointValues(roboplan_trajectory.positions, i);
    const auto* velocities = pointValues(roboplan_trajectory.velocities, i);
    const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i);
    size_t q_idx = 0;
    size_t v_idx = 0;
    for (size_t j = 0; j < joints->size(); ++j) {
      const auto& joint = joints->at(j);
      if (in_message(joint)) {
        if (positions && joint.kind == TrajectoryJointKind::CONTINUOUS) {
          const auto angle = unwrapAngle(std::atan2((*positions)(q_idx + 1), (*positions)(q_idx)),
                                         previous_angles[j]);
          previous_angles[j] = angle;
          point.positions.push_back(angle);
        } else if (positions) {
          point.positions.push_back((*positions)(q_idx));
        }
        if (velocities) {
          point.velocities.push_back((*velocities)(v_idx));
        }
        if (accelerations) {
          point.accelerations.push_back((*accelerations)(v_idx));
        }
      }
      q_idx += joint.nq;
      v_idx += joint.nv;
    }
  }
  return ros_traj;
}

tl::expected<trajectory_msgs::msg::MultiDOFJointTrajectory, std::string>
toMultiDOFJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                          const roboplan::Scene& scene) {
  const auto joints = trajectoryJoints(scene, roboplan_trajectory.joint_names);
  if (!joints) {
    return tl::make_unexpected("Failed to convert the trajectory: " + joints.error());
  }
  const auto size = trajectorySize(joints.value());

  trajectory_msgs::msg::MultiDOFJointTrajectory ros_traj;
  for (const auto& joint : joints.value()) {
    if (joint.isMultiDof()) {
      ros_traj.joint_names.push_back(joint.name);
    }
  }
  if (ros_traj.joint_names.empty()) {
    return ros_traj;
  }

  std::vector<std::optional<Eigen::Quaterniond>> previous_rotations(joints->size());
  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < ros_traj.points.size(); ++i) {
    if (auto ok = checkRoboPlanPoint(roboplan_trajectory, i, size); !ok) {
      return tl::make_unexpected(ok.error());
    }
    auto& point = ros_traj.points[i];
    point.time_from_start = toDuration(roboplan_trajectory.times[i]);
    const auto* positions = pointValues(roboplan_trajectory.positions, i);
    const auto* velocities = pointValues(roboplan_trajectory.velocities, i);
    const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i);
    size_t q_idx = 0;
    size_t v_idx = 0;
    for (size_t j = 0; j < joints->size(); ++j) {
      const auto& joint = joints->at(j);
      if (joint.isMultiDof()) {
        if (positions) {
          const auto& q = *positions;
          geometry_msgs::msg::Transform transform;
          Eigen::Quaterniond rotation;
          if (joint.kind == TrajectoryJointKind::PLANAR) {
            transform.translation.x = q(q_idx);
            transform.translation.y = q(q_idx + 1);
            const auto angle = std::atan2(q(q_idx + 3), q(q_idx + 2));
            rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
          } else {
            transform.translation.x = q(q_idx);
            transform.translation.y = q(q_idx + 1);
            transform.translation.z = q(q_idx + 2);
            rotation = Eigen::Quaterniond(q(q_idx + 6), q(q_idx + 3), q(q_idx + 4), q(q_idx + 5));
          }
          // q and -q are the same rotation: keep the sign closest to the previous point, so that
          // interpolating between consecutive points takes the short way.
          if (previous_rotations[j] && previous_rotations[j]->dot(rotation) < 0.0) {
            rotation.coeffs() *= -1.0;
          }
          previous_rotations[j] = rotation;
          transform.rotation = tf2::toMsg(rotation);
          point.transforms.push_back(transform);
        }
        if (velocities) {
          point.velocities.push_back(toTwist(joint.kind, *velocities, v_idx));
        }
        if (accelerations) {
          point.accelerations.push_back(toTwist(joint.kind, *accelerations, v_idx));
        }
      }
      q_idx += joint.nq;
      v_idx += joint.nv;
    }
  }
  return ros_traj;
}

tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const roboplan::Scene& scene) {
  return fromRosTrajectories(ros_trajectory, trajectory_msgs::msg::MultiDOFJointTrajectory(), scene,
                             false);
}

tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene) {
  return fromRosTrajectories(ros_trajectory, multi_dof_trajectory, scene, true);
}

geometry_msgs::msg::TransformStamped
toTransformStamped(const roboplan::CartesianConfiguration& cartesian_configuration) {
  Eigen::Isometry3d isometry(cartesian_configuration.tform);
  geometry_msgs::msg::TransformStamped ret = tf2::eigenToTransform(isometry);
  ret.header.frame_id = cartesian_configuration.base_frame;
  ret.child_frame_id = cartesian_configuration.tip_frame;
  return ret;
}

roboplan::CartesianConfiguration
fromTransformStamped(const geometry_msgs::msg::TransformStamped& transform_stamped) {
  roboplan::CartesianConfiguration config;
  config.base_frame = transform_stamped.header.frame_id;
  config.tip_frame = transform_stamped.child_frame_id;
  Eigen::Isometry3d isometry = tf2::transformToEigen(transform_stamped.transform);
  config.tform = isometry.matrix();
  return config;
}

Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d isometry;
  tf2::fromMsg(pose, isometry);
  return isometry.matrix();
}

geometry_msgs::msg::Pose se3ToPose(const Eigen::Matrix4d& transform) {
  Eigen::Isometry3d isometry(transform);
  return tf2::toMsg(isometry);
}

}  // namespace roboplan_ros_cpp
