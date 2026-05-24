from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_low_level_policy_profile():
    return "push_low_level"


def get_default_cmd_vel_topic():
    return "/push_cmd_vel"


def get_default_converted_push_obs_topic():
    return "/push_box_obs_float"


def get_high_level_policy_path_parts():
    return ("config", "push_policy", "policy.onnx")


def get_high_level_start_delay_sec():
    return 5.0


def get_high_level_policy_path():
    return PathJoinSubstitution(
        [FindPackageShare("go2_description"), *get_high_level_policy_path_parts()]
    )


def generate_launch_description():
    bringup_rl_launch = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "launch", "bringup_rl.launch.py"]
    )
    push_controller_launch = PathJoinSubstitution(
        [FindPackageShare("push_controller"), "launch", "push_controller.launch.py"]
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
            "push_obs_topic",
            default_value=get_default_converted_push_obs_topic(),
            description="16D Float32MultiArray push-box observation topic for push_controller.",
        ),
        DeclareLaunchArgument(
            "raw_push_obs_topic",
            default_value="/push_box_obs",
            description="Raw unitree_go/HeightMap push-box observation topic from MuJoCo/Unitree DDS.",
        ),
        DeclareLaunchArgument(
            "converted_push_obs_topic",
            default_value=get_default_converted_push_obs_topic(),
            description="Converted std_msgs/Float32MultiArray push-box observation topic.",
        ),
        DeclareLaunchArgument(
            "goal_pose_topic",
            default_value="/push_box_goal_pose",
            description="Fallback push-box goal pose topic.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value=get_default_cmd_vel_topic(),
            description="Push velocity command topic shared by push_controller and rl_controller.",
        ),
        DeclareLaunchArgument(
            "high_level_onnx_model_path",
            default_value=get_high_level_policy_path(),
            description="Path to the high-level push-box ONNX policy.",
        ),
    ]

    low_level_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_rl_launch),
        launch_arguments={
            "policy_profile": get_low_level_policy_profile(),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_rqt_cm": LaunchConfiguration("use_rqt_cm"),
            "enable_lowlevel_write": LaunchConfiguration("enable_lowlevel_write"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
        }.items(),
    )

    push_obs_bridge = Node(
        package="push_controller",
        executable="push_box_obs_bridge_node",
        name="push_box_obs_bridge_node",
        output="screen",
        parameters=[{
            "input_topic": LaunchConfiguration("raw_push_obs_topic"),
            "output_topic": LaunchConfiguration("converted_push_obs_topic"),
        }],
    )

    high_level_push = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(push_controller_launch),
        launch_arguments={
            "onnx_model_path": LaunchConfiguration("high_level_onnx_model_path"),
            "push_obs_topic": LaunchConfiguration("push_obs_topic"),
            "goal_pose_topic": LaunchConfiguration("goal_pose_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
        }.items(),
    )

    delayed_high_level_push = TimerAction(
        period=get_high_level_start_delay_sec(),
        actions=[push_obs_bridge, high_level_push],
    )

    return LaunchDescription(declared_arguments + [low_level_bringup, delayed_high_level_push])
