#!/usr/bin/env python3
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bishe_bridge.common import ControlFileWriter


class SkillCommandNode(Node):
    def __init__(self):
        super().__init__("go2_skill_command_node")
        self.declare_parameter("skill_command_topic", "/go2/skill_command")
        self.declare_parameter("control_dir", "/tmp/mujoco_go2_control")
        self.writer = ControlFileWriter(str(self.get_parameter("control_dir").value))
        self.create_subscription(String, str(self.get_parameter("skill_command_topic").value), self.on_skill_command, 10)

    def on_skill_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignore invalid skill_command: {msg.data!r}")
            return
        if not isinstance(payload, dict):
            self.get_logger().warn(f"Ignore non-object skill_command: {msg.data!r}")
            return
        self.writer.write_skill_command(payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SkillCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
