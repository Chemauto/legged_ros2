import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("nav_controller")

    # Default ONNX model path: relative to the workspace src directory
    default_model_path = os.path.join(
        pkg_dir, "..", "..", "..", "..",
        "src", "legged_ros2",
        "legged_robot_description", "go2_description", "config", "nav_policy", "policy.onnx"
    )

    config_file = os.path.join(pkg_dir, "config", "nav_controller.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "onnx_model_path",
            default_value=default_model_path,
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
