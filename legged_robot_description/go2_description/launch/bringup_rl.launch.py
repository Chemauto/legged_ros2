import os
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


@dataclass(frozen=True)
class PolicyProfilePaths:
    policy_dir_parts: tuple[str, ...]


_POLICY_PROFILE_PATHS = {
    "rl": PolicyProfilePaths(("rl_policy",)),
    "nav_low_level": PolicyProfilePaths(("nav_policy", "low_level_policy")),
    "push_low_level": PolicyProfilePaths(("push_policy", "low_level_policy")),
}


def get_policy_profile_paths(policy_profile):
    try:
        return _POLICY_PROFILE_PATHS[policy_profile]
    except KeyError:
        supported_profiles = ", ".join(sorted(_POLICY_PROFILE_PATHS))
        raise ValueError(
            f"Unsupported policy_profile '{policy_profile}'. "
            f"Supported profiles: {supported_profiles}"
        ) from None


def resolve_policy_file_path(context, launch_argument_name, filename):
    explicit_path = LaunchConfiguration(launch_argument_name).perform(context).strip()
    if explicit_path:
        return explicit_path

    policy_profile = LaunchConfiguration("policy_profile").perform(context).strip()
    policy_paths = get_policy_profile_paths(policy_profile)
    policy_dir = os.path.join(
        get_package_share_directory("go2_description"),
        "config",
        *policy_paths.policy_dir_parts,
    )
    return os.path.join(policy_dir, filename)


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controller_config = LaunchConfiguration("controller_config")
    main_loop_config = LaunchConfiguration("main_loop_config")
    enable_lowlevel_write = LaunchConfiguration("enable_lowlevel_write")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    onnx_model_path = resolve_policy_file_path(context, "onnx_model_path", "policy.onnx")
    io_descriptors_path = resolve_policy_file_path(
        context, "io_descriptors_path", "IO_descriptors.yaml")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rqt_cm = LaunchConfiguration("use_rqt_cm")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "enable_sim:=",
            "false",
            " ",
            "enable_lowlevel_write:=",
            enable_lowlevel_write,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "ros2_control",
            controller_config,
        ]
    )

    main_loop_config_path = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            "main_loop",
            main_loop_config,
        ]
    )

    rl_controller_params = {
        "onnx_model_path": onnx_model_path,
        "io_descriptors_path": io_descriptors_path,
        "cmd_vel_topic": cmd_vel_topic,
    }

    main_loop_node = Node(
        package="legged_ros2_control",
        executable="go2_main_loop",
        parameters=[
            controller_config_path,
            robot_description,
            main_loop_config_path,
            rl_controller_params,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # -----------------------------------------------------------------------
    # RVIZ
    # -----------------------------------------------------------------------
    pkg_share = get_package_share_directory("go2_description")
    rviz_config_file = os.path.join(pkg_share, "rviz2", "go2.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
    )

    rqt_controller_manager = Node(
        package="rqt_controller_manager",
        executable="rqt_controller_manager",
        condition=IfCondition(use_rqt_cm),
    )

    # -----------------------------------------------------------------------
    # Spawners
    # -----------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    imu_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rl_controller", "-c", "/controller_manager", "--inactive"],
    )

    stand_static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["stand_static_controller", "-c", "/controller_manager", "--inactive"],
    )

    sit_static_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sit_static_controller", "-c", "/controller_manager", "--inactive"],
    )

    spawn_joint_state_broadcaster_after_stand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=stand_static_controller_spawner,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    spawn_imu_state_broadcaster_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                imu_state_broadcaster_spawner,
            ],
        )
    )

    spawn_rl_controller_after_imu_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_state_broadcaster_spawner,
            on_exit=[
                rl_controller_spawner,
            ],
        )
    )

    spawn_sit_static_controller_after_rl_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rl_controller_spawner,
            on_exit=[
                sit_static_controller_spawner,
            ],
        )
    )

    start_optional_tools_after_sit_static_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=sit_static_controller_spawner,
            on_exit=[
                rviz_node,
                rqt_controller_manager,
            ],
        )
    )

    return [
        main_loop_node,
        robot_state_pub_node,
        stand_static_controller_spawner,
        spawn_joint_state_broadcaster_after_stand,
        spawn_imu_state_broadcaster_after_joint_state,
        spawn_rl_controller_after_imu_state,
        spawn_sit_static_controller_after_rl_controller,
        start_optional_tools_after_sit_static_controller,
    ]


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="go2_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_config",
            default_value="rl.yaml",
            description="Controller configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "main_loop_config",
            default_value="rl.yaml",
            description="Main loop configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_lowlevel_write",
            default_value="true",
            description="Enable low-level command writing, useful in debugging or testing scenarios. \
                        If set to true, the robot will receive low-level commands from the controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="cmd_vel",
            description="Velocity command topic consumed by rl_controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "policy_profile",
            default_value="rl",
            description="Policy profile to load when explicit policy paths are not provided. "
                        "Supported values: rl, nav_low_level, push_low_level.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value="",
            description="Path to ONNX policy model for RL controller. "
                        "Overrides policy_profile when set.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "io_descriptors_path",
            default_value="",
            description="Path to IO descriptors YAML for RL controller. "
                        "Overrides policy_profile when set.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rqt_cm",
            default_value="false",
            description="Start rqt_controller_manager automatically with this launch file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
