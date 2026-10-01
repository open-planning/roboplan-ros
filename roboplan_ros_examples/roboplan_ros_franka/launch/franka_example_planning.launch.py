from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from roboplan.example_models import get_package_models_dir, get_package_share_dir

FR3_MODELS_DIR = get_package_models_dir() / "franka_robot_model"


def generate_launch_description():
    pkg_share = get_package_share_path("roboplan_ros_franka")
    hardware_type = LaunchConfiguration("hardware_type")
    use_mujoco = PythonExpression(["'", hardware_type, "' == 'mujoco'"])

    # In MuJoCo, the tabletop is also added to the planning scene.
    obstacles_config_file = (
        pkg_share / "config" / "fr3_mujoco_obstacles.yaml"
    ).as_posix()
    obstacles_config_file = PythonExpression(
        ["'", obstacles_config_file, "' if ", use_mujoco, " else ''"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_type",
                default_value="mock",
                description="The ros2_control hardware to use ('mock' or 'mujoco').",
            ),
            # Include the control launch file with custom rviz config
            IncludeLaunchDescription(
                (pkg_share / "launch" / "franka_control.launch.py").as_posix(),
                launch_arguments={
                    "rviz_config": (
                        pkg_share / "config" / "franka_ik_config.rviz"
                    ).as_posix(),
                    "hardware_type": hardware_type,
                }.items(),
            ),
            # IK planning and execution node.
            Node(
                package="roboplan_ros_franka",
                executable="plan_and_execute_node.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_mujoco, value_type=bool),
                        "srdf_file": (FR3_MODELS_DIR / "fr3.srdf").as_posix(),
                        "yaml_config_file": (
                            FR3_MODELS_DIR / "fr3_config.yaml"
                        ).as_posix(),
                        "package_paths": [get_package_share_dir().as_posix()],
                        "obstacles_config_file": ParameterValue(
                            obstacles_config_file, value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
