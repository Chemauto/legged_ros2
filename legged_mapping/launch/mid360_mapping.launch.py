from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("input_topic", default_value="livox/imu"),
        DeclareLaunchArgument("output_topic", default_value="mid360/imu"),
        DeclareLaunchArgument("queue_size", default_value="50"),
        DeclareLaunchArgument("base_imu_topic", default_value="lowstate"),
        DeclareLaunchArgument("lidar_imu_topic", default_value="/mid360/imu"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("camera_init_frame", default_value="camera_init"),
        DeclareLaunchArgument("lidar_frame", default_value="body"),
        DeclareLaunchArgument("base_frame", default_value="base"),
        DeclareLaunchArgument("t_base_to_lidar", default_value="[0.1870, 0.0, 0.0803]"),
        DeclareLaunchArgument("init_window", default_value="200"),
    ]

    mid360_imu_node = Node(
        package="legged_mapping",
        executable="mid360_imu_node",
        output="both",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "queue_size": LaunchConfiguration("queue_size"),
            }
        ],
    )

    dual_imu_static_tf_node = Node(
        package="legged_mapping",
        executable="dual_imu_static_tf_node",
        output="both",
        parameters=[
            {
                "base_imu_topic": LaunchConfiguration("base_imu_topic"),
                "lidar_imu_topic": LaunchConfiguration("lidar_imu_topic"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "camera_init_frame": LaunchConfiguration("camera_init_frame"),
                "lidar_frame": LaunchConfiguration("lidar_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "t_base_to_lidar": LaunchConfiguration("t_base_to_lidar"),
                "init_window": LaunchConfiguration("init_window"),
                "queue_size": LaunchConfiguration("queue_size"),
            }
        ],
    )

    start_dual_imu_static_tf_after_mid360 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mid360_imu_node,
            on_start=[dual_imu_static_tf_node],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            mid360_imu_node,
            start_dual_imu_static_tf_after_mid360,
        ]
    )
