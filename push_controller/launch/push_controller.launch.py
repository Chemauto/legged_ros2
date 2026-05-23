from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_default_model_path_parts():
    return ("config", "push_policy", "policy.onnx")


def get_default_model_path():
    return PathJoinSubstitution(
        [FindPackageShare("go2_description"), *get_default_model_path_parts()]
    )


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("push_controller"), "config", "push_controller.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value=get_default_model_path(),
            description="Path to the high-level push-box ONNX policy.",
        ),
        DeclareLaunchArgument(
            "push_obs_topic",
            default_value="/push_box_obs",
            description="16D external push-box observation topic.",
        ),
        DeclareLaunchArgument(
            "goal_pose_topic",
            default_value="/push_box_goal_pose",
            description="Fallback goal pose topic when external push observations are disabled.",
        ),
        Node(
            package="push_controller",
            executable="push_controller_node",
            name="push_controller_node",
            output="screen",
            parameters=[
                config_file,
                {"onnx_model_path": LaunchConfiguration("onnx_model_path")},
                {"push_obs_topic": LaunchConfiguration("push_obs_topic")},
                {"goal_pose_topic": LaunchConfiguration("goal_pose_topic")},
            ],
        ),
    ])
