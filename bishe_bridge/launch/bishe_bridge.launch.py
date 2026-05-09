from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _node(executable: str, name: str, parameters: list[dict], arguments: list | None = None) -> Node:
    return Node(
        package="bishe_bridge",
        executable=executable,
        name=name,
        output="screen",
        parameters=parameters,
        arguments=arguments or [],
        additional_env={"CYCLONEDDS_URI": ""},
    )


def generate_launch_description() -> LaunchDescription:
    domain_id = LaunchConfiguration("domain_id")
    interface = LaunchConfiguration("interface")
    publish_hz = LaunchConfiguration("publish_hz")
    frame_id = LaunchConfiguration("frame_id")
    child_frame_id = LaunchConfiguration("child_frame_id")
    control_dir = LaunchConfiguration("control_dir")
    state_file = LaunchConfiguration("state_file")
    rl_cmd_vel_topic = LaunchConfiguration("rl_cmd_vel_topic")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    static_scene_objects = LaunchConfiguration("static_scene_objects")

    odom_topic = LaunchConfiguration("odom_topic")
    box_pose_topic = LaunchConfiguration("box_pose_topic")
    skill_status_topic = LaunchConfiguration("skill_status_topic")
    scene_objects_topic = LaunchConfiguration("scene_objects_topic")
    skill_command_topic = LaunchConfiguration("skill_command_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    goal_pose_topic = LaunchConfiguration("goal_pose_topic")

    dds_params = {
        "domain_id": domain_id,
        "interface": interface,
        "publish_hz": publish_hz,
    }

    return LaunchDescription([
        DeclareLaunchArgument("domain_id", default_value="0"),
        DeclareLaunchArgument("interface", default_value="lo"),
        DeclareLaunchArgument("publish_hz", default_value="10.0"),
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("child_frame_id", default_value="base"),
        DeclareLaunchArgument("control_dir", default_value="/tmp/mujoco_go2_control"),
        DeclareLaunchArgument("state_file", default_value="/tmp/bishe_bridge_state.json"),
        DeclareLaunchArgument("rl_cmd_vel_topic", default_value="rl_cmd_vel"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.08"),
        DeclareLaunchArgument("static_scene_objects", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="/go2/odom"),
        DeclareLaunchArgument("box_pose_topic", default_value="/go2/box_pose"),
        DeclareLaunchArgument("skill_status_topic", default_value="/go2/skill_status"),
        DeclareLaunchArgument("scene_objects_topic", default_value="/go2/scene_objects"),
        DeclareLaunchArgument("skill_command_topic", default_value="/go2/skill_command"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/go2/cmd_vel"),
        DeclareLaunchArgument("goal_pose_topic", default_value="/go2/goal_pose"),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "bishe_bridge", "dds_state_writer.py",
                "--domain-id", domain_id,
                "--interface", interface,
                "--state-file", state_file,
            ],
            name="dds_state_writer",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "run", "bishe_bridge", "rl_cmd_vel_writer.py",
                "--control-dir", control_dir,
                "--topic", rl_cmd_vel_topic,
            ],
            name="rl_cmd_vel_writer",
            output="screen",
        ),
        _node("state_bridge_node.py", "go2_state_bridge_node", [{
            "odom_topic": odom_topic,
            "box_pose_topic": box_pose_topic,
            "scene_objects_topic": scene_objects_topic,
            "skill_status_topic": skill_status_topic,
            "skill_command_topic": skill_command_topic,
            "frame_id": frame_id,
            "child_frame_id": child_frame_id,
            "static_scene_objects": static_scene_objects,
            "goal_tolerance": goal_tolerance,
            "state_file": state_file,
            "publish_hz": publish_hz,
        }]),
        _node("skill_command_node.py", "go2_skill_command_node", [{
            "skill_command_topic": skill_command_topic,
            "control_dir": control_dir,
        }]),
        _node("cmd_vel_node.py", "go2_cmd_vel_node", [{
            "cmd_vel_topic": cmd_vel_topic,
            "control_dir": control_dir,
        }]),
        _node("goal_pose_node.py", "go2_goal_pose_node", [{
            "goal_pose_topic": goal_pose_topic,
            "control_dir": control_dir,
        }]),
    ])
