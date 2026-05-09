#!/usr/bin/env python3
from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from bishe_bridge.common import (
    SkillStatusTracker,
    box_world_position_from_snapshot,
    build_scene_objects,
    parse_scene_objects_config,
    quat_wxyz_to_xyzw,
)
from bishe_bridge.ros_params import declare_publish_hz
from bishe_bridge.state_file import read_state_file


class StateBridgeNode(Node):
    """Publish all /go2 state topics from one Unitree DDS participant."""

    def __init__(self):
        super().__init__("go2_state_bridge_node")
        self.declare_parameter("odom_topic", "/go2/odom")
        self.declare_parameter("box_pose_topic", "/go2/box_pose")
        self.declare_parameter("scene_objects_topic", "/go2/scene_objects")
        self.declare_parameter("skill_status_topic", "/go2/skill_status")
        self.declare_parameter("skill_command_topic", "/go2/skill_command")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base")
        self.declare_parameter("static_scene_objects", "")
        self.declare_parameter("goal_tolerance", 0.08)
        self.declare_parameter("state_file", "/tmp/bishe_bridge_state.json")

        publish_hz = declare_publish_hz(self)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.state_file = str(self.get_parameter("state_file").value)
        self.static_objects = parse_scene_objects_config(str(self.get_parameter("static_scene_objects").value))
        self.skill_status = SkillStatusTracker(goal_tolerance=float(self.get_parameter("goal_tolerance").value))

        self.odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.box_pose_pub = self.create_publisher(PoseStamped, str(self.get_parameter("box_pose_topic").value), 10)
        self.scene_objects_pub = self.create_publisher(String, str(self.get_parameter("scene_objects_topic").value), 10)
        self.skill_status_pub = self.create_publisher(String, str(self.get_parameter("skill_status_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("skill_command_topic").value), self.on_skill_command, 10)
        self.create_timer(1.0 / publish_hz, self.publish_state)

    def on_skill_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignore invalid skill_command: {msg.data!r}")
            return
        if isinstance(payload, dict):
            self.skill_status.apply_command(payload, timestamp=time.time())

    def publish_state(self) -> None:
        snap = read_state_file(self.state_file)
        self.publish_odom(snap)
        self.publish_box_pose(snap)
        self.publish_scene_objects(snap)
        self.publish_skill_status(snap)

    def publish_odom(self, snap: dict) -> None:
        pos = snap.get("robot_pos")
        if pos is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        quat = snap.get("robot_quat")
        if quat and len(quat) >= 4:
            x, y, z, w = quat_wxyz_to_xyzw(quat)
            msg.pose.pose.orientation.x = x
            msg.pose.pose.orientation.y = y
            msg.pose.pose.orientation.z = z
            msg.pose.pose.orientation.w = w
        else:
            msg.pose.pose.orientation.w = 1.0
        vel = snap.get("robot_vel")
        if vel:
            msg.twist.twist.linear.x = float(vel[0])
            msg.twist.twist.linear.y = float(vel[1])
            msg.twist.twist.linear.z = float(vel[2])
        self.odom_pub.publish(msg)

    def publish_box_pose(self, snap: dict) -> None:
        box_pos = box_world_position_from_snapshot(snap)
        if box_pos is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = box_pos[0]
        msg.pose.position.y = box_pos[1]
        msg.pose.position.z = box_pos[2]
        msg.pose.orientation.w = 1.0
        self.box_pose_pub.publish(msg)

    def publish_scene_objects(self, snap: dict) -> None:
        msg = String()
        msg.data = json.dumps(build_scene_objects(snap, self.static_objects), ensure_ascii=False)
        self.scene_objects_pub.publish(msg)

    def publish_skill_status(self, snap: dict) -> None:
        msg = String()
        msg.data = json.dumps(self.skill_status.status(snap), ensure_ascii=False)
        self.skill_status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
