from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_default_model_path_parts():
    return ("config", "nav_policy", "policy.onnx")


def get_default_model_path():
    return PathJoinSubstitution(
        [FindPackageShare("go2_description"), *get_default_model_path_parts()]
    )


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("nav_controller"), "config", "nav_controller.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value=get_default_model_path(),
            description="Path to the high-level navigation ONNX policy",
        ),
        DeclareLaunchArgument(
            "goal_pose_topic",
            default_value="/go2/goal_pose",
            description="Topic for goal pose commands",
        ),
        Node(
            package="nav_controller",
            executable="nav_controller_node",
            name="nav_controller_node",
            output="screen",
            parameters=[
                config_file,
                {"onnx_model_path": LaunchConfiguration("onnx_model_path")},
                {"goal_pose_topic": LaunchConfiguration("goal_pose_topic")},
            ],
        ),
    ])
