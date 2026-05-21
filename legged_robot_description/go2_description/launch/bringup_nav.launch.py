from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def get_low_level_policy_profile():
    return "nav_low_level"


def get_high_level_policy_path_parts():
    return ("config", "nav_policy", "policy.onnx")


def get_high_level_policy_path():
    return PathJoinSubstitution(
        [FindPackageShare("go2_description"), *get_high_level_policy_path_parts()]
    )


def generate_launch_description():
    bringup_rl_launch = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "launch", "bringup_rl.launch.py"]
    )
    nav_controller_launch = PathJoinSubstitution(
        [FindPackageShare("nav_controller"), "launch", "nav_controller.launch.py"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 with the low-level bringup.",
        ),
        DeclareLaunchArgument(
            "use_rqt_cm",
            default_value="false",
            description="Start rqt_controller_manager with the low-level bringup.",
        ),
        DeclareLaunchArgument(
            "enable_lowlevel_write",
            default_value="true",
            description="Enable low-level command writing.",
        ),
        DeclareLaunchArgument(
            "goal_pose_topic",
            default_value="/go2/goal_pose",
            description="Topic for high-level navigation goal poses.",
        ),
        DeclareLaunchArgument(
            "high_level_onnx_model_path",
            default_value=get_high_level_policy_path(),
            description="Path to the high-level navigation ONNX policy.",
        ),
    ]

    low_level_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_rl_launch),
        launch_arguments={
            "policy_profile": get_low_level_policy_profile(),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_rqt_cm": LaunchConfiguration("use_rqt_cm"),
            "enable_lowlevel_write": LaunchConfiguration("enable_lowlevel_write"),
        }.items(),
    )

    high_level_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_controller_launch),
        launch_arguments={
            "onnx_model_path": LaunchConfiguration("high_level_onnx_model_path"),
            "goal_pose_topic": LaunchConfiguration("goal_pose_topic"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [low_level_bringup, high_level_nav])
