import xacro

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from roboplan.example_models import get_package_models_dir, get_package_share_dir

FR3_URDF = get_package_models_dir() / "franka_robot_model" / "fr3.urdf"


def launch_setup(context):
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    use_mujoco = hardware_type == "mujoco"

    pkg_share = get_package_share_path("roboplan_ros_franka")
    controllers_file = (pkg_share / "config" / "ros2_controllers.yaml").as_posix()

    # The robot description is only set here; everything else gets it from the
    # /robot_description topic.
    robot_description = xacro.process_file(
        FR3_URDF.as_posix(), mappings={"ros2_control_hardware": hardware_type}
    ).toxml()

    # Replace package:// mesh URIs with absolute file:// URIs, since
    # roboplan_example_models may not be discoverable through the ament index
    # (e.g., when installed from conda).
    robot_description = robot_description.replace(
        "package://roboplan_example_models/",
        (get_package_share_dir() / "roboplan_example_models").as_uri() + "/",
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {"use_sim_time": use_mujoco, "robot_description": robot_description}
            ],
        ),
    ]

    if use_mujoco:
        # Auto-generate the MuJoCo model (MJCF) from the published robot description,
        # merge in the tabletop scene, and publish it for the simulator to load.
        # Note this can take a while when first launching.
        nodes.append(
            Node(
                package="mujoco_ros2_control",
                executable="robot_description_to_mjcf.sh",
                output="screen",
                arguments=[
                    "-m",
                    (pkg_share / "description" / "fr3_mujoco_inputs.xml").as_posix(),
                    "--scene",
                    (pkg_share / "description" / "fr3_mujoco_scene.xml").as_posix(),
                    "--publish_topic",
                    "/mujoco_robot_description",
                ],
            )
        )
        # The MuJoCo simulation, which also runs the controller manager
        nodes.append(
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                output="screen",
                parameters=[{"use_sim_time": True}, controllers_file],
                remappings=[("~/robot_description", "/robot_description")],
            )
        )
    else:
        # Controller manager with mock hardware
        nodes.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[controllers_file],
                remappings=[("~/robot_description", "/robot_description")],
            )
        )

    for controller in [
        "joint_state_broadcaster",
        "fr3_arm_controller",
        "fr3_gripper_controller",
    ]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    controller,
                    "--controller-manager-timeout",
                    "300",
                    "--param-file",
                    controllers_file,
                ],
            )
        )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": use_mujoco}],
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=(
                    get_package_share_path("roboplan_ros_franka")
                    / "config"
                    / "franka_config.rviz"
                ).as_posix(),
                description="Specify an rviz configuration file.",
            ),
            DeclareLaunchArgument(
                "hardware_type",
                default_value="mock",
                description="The ros2_control hardware to use ('mock' or 'mujoco').",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
