#pragma once

#include <optional>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tl/expected.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

namespace roboplan_ros_cpp {

/// @brief Pre-computed mapping for ROS JointState to RoboPlan type conversions.
/// @details JointState messages may be in different orders, will not contain mimic states, and
/// have different representation of continuous types. This structure maintains a mapping
/// from ROS JointStates to RoboPlan JointConfigurations to enable efficient conversion
/// from one type to the other.
struct JointStateConverterMap {
  /// @brief Mapping for an individual joint
  struct JointMapping {
    /// @brief the String name of the joint.
    std::string joint_name;

    /// @brief Index in the ROS JointState type.
    size_t ros_index;

    /// @brief The start index in the positions vector.
    size_t q_start;

    /// @brief The start index in velocities vector.
    size_t v_start;

    /// @brief The RoboPlan type of the Joint.
    roboplan::JointType type;
  };

  /// @brief Index of JointState joints in the Scene.
  std::vector<JointMapping> mappings;

  /// @brief Number of position states in the Scene.
  size_t nq;

  /// @brief Numbef of velocity states in the Scene.
  size_t nv;
};

/// @brief Constructs a JointState conversion map given a RoboPlan scene and JointState message.
/// @param scene The RoboPlan Scene.
/// @param joint_state Sample ROS joint_state message.
/// @return The joint information struct if successful, else a string describing the error.
tl::expected<JointStateConverterMap, std::string>
buildConversionMap(const roboplan::Scene& scene, const sensor_msgs::msg::JointState& joint_state);

/// @brief Converts a double timestamp to an equivalent ROS Duration.
/// @param time_sec Timestamp in seconds.
/// @return ROS 2 Duration with seconds and nanoseconds.
inline builtin_interfaces::msg::Duration toDuration(const double time_sec) {
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(time_sec);
  duration.nanosec = static_cast<uint32_t>((time_sec - duration.sec) * 1e9);
  return duration;
}

/// @brief Converts a ROS 2 Duration with seconds and nanoseconds to an equivalent double timestamp.
/// @param duration ROS 2 Duration with seconds and nanoseconds.
/// @return An equivalent double timestamp.
inline double fromDuration(const builtin_interfaces::msg::Duration& duration) {
  return duration.sec + duration.nanosec * 1e-9;
}

/// @brief Converts a roboplan::JointConfiguration object to a ROS 2 JointState message.
/// @param config The roboplan JointConfiguration to convert
/// @param scene The RoboPlan scene containing the model's joint information.
/// @return An equivalent ROS 2 JointState message, or an error.
tl::expected<sensor_msgs::msg::JointState, std::string>
toJointState(const roboplan::JointConfiguration& config, const roboplan::Scene& scene);

/// @brief Convert the provided ROS 2 JointState message to an equivalent
/// roboplan::JointConfiguration.
/// @details Joints that are not in the message keep their positions from `q_reference`, or from
/// the scene's current state if it is not given. This matters most for continuous and planar
/// joints, whose positions are stored as cos and sin and would be invalid if zeroed.
/// @param joint_state The ROS 2 JointState message to convert.
/// @param scene The RoboPlan scene containing the model's joint information.
/// @param joint_conversion_map A mapping of joint names to their indexes in the model.
/// @param q_reference The full configuration supplying the joints that are not in the message.
/// @return An equivalent roboplan::JointConfiguration, or an error.
tl::expected<roboplan::JointConfiguration, std::string>
fromJointState(const sensor_msgs::msg::JointState& joint_state, const roboplan::Scene& scene,
               const JointStateConverterMap& joint_conversion_map,
               const std::optional<Eigen::VectorXd>& q_reference = std::nullopt);

/// @brief Converts the single degree of freedom joints of a roboplan::JointTrajectory to a ROS 2
/// JointTrajectory message.
/// @details Continuous joints become angles, unwrapped along the trajectory. Mimic joints are left
/// out, and so are planar and floating joints: use toMultiDOFJointTrajectory for them.
/// @param roboplan_trajectory The roboplan JointTrajectory to convert.
/// @param scene The RoboPlan scene containing the model's joint information.
/// @return An equivalent ROS 2 JointTrajectory message, or an error.
tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene);

/// @brief Converts the planar and floating joints of a roboplan::JointTrajectory to a ROS 2
/// MultiDOFJointTrajectory message.
/// @details One transform per joint and point. Velocities and accelerations are twists in the
/// joint's own frame, as in RoboPlan. The message is empty without planar or floating joints.
/// @param roboplan_trajectory The roboplan JointTrajectory to convert.
/// @param scene The RoboPlan scene containing the model's joint information.
/// @return An equivalent ROS 2 MultiDOFJointTrajectory message, or an error.
tl::expected<trajectory_msgs::msg::MultiDOFJointTrajectory, std::string>
toMultiDOFJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                          const roboplan::Scene& scene);

/// @brief Converts a ROS 2 JointTrajectory message to a roboplan::JointTrajectory.
/// @details The values of mimic joints are ignored. Planar and floating joints are an error: use
/// the overload that also takes a MultiDOFJointTrajectory.
/// @param ros_trajectory The ROS 2 JointTrajectory message to convert.
/// @param scene The RoboPlan scene containing the model's joint information.
/// @return An equivalent roboplan::JointTrajectory, or an error.
tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const roboplan::Scene& scene);

/// @brief Converts a ROS 2 JointTrajectory and MultiDOFJointTrajectory pair, as made by
/// toJointTrajectory and toMultiDOFJointTrajectory, to one roboplan::JointTrajectory.
/// @details Either message may have no joints; otherwise both must have the same points at the
/// same times. The joints of the JointTrajectory come first, then those of the
/// MultiDOFJointTrajectory, each in their message's order.
/// @param ros_trajectory The ROS 2 JointTrajectory message with the single DOF joints.
/// @param multi_dof_trajectory The ROS 2 MultiDOFJointTrajectory with the planar and floating
/// joints.
/// @param scene The RoboPlan scene containing the model's joint information.
/// @return An equivalent roboplan::JointTrajectory, or an error.
tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene);

/// @brief Converts a roboplan::CartesianConfiguration to a ROS 2 TransformStamped message.
/// @param cartesian_configuration The roboplan CartesianConfiguration to convert.
/// @return An equivalent geometry_msgs::msg::TransformStamped.
geometry_msgs::msg::TransformStamped
toTransformStamped(const roboplan::CartesianConfiguration& cartesian_configuration);

/// @brief Converts a geometry_msgs::msg::TransformStamped to a roboplan CartesianConfiguration.
/// @param transform The ROS 2 TransformStamped message to convert.
/// @return An equivalent roboplan::CartesianConfiguration.
roboplan::CartesianConfiguration
fromTransformStamped(const geometry_msgs::msg::TransformStamped& transform);

/// @brief Convert ROS Pose to a 4x4 SE3 transformation matrix.
/// @param pose ROS Pose message
/// @return 4x4 Eigen matrix representing the SE3 transformation
Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& pose);

/// @brief Convert a 4x4 SE3 transformation matrix to ROS Pose.
/// @param transform 4x4 Eigen matrix representing an SE3 transformation
/// @return An equivalent ROS Pose message
geometry_msgs::msg::Pose se3ToPose(const Eigen::Matrix4d& transform);

}  // namespace roboplan_ros_cpp
