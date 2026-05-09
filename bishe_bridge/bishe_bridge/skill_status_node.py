#!/usr/bin/env python3
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bishe_bridge.common import SkillStatusTracker
from bishe_bridge.dds_state import DdsStateCache, start_dds_subscribers
from bishe_bridge.ros_params import declare_common_dds_params, declare_publish_hz


class SkillStatusNode(Node):
    def __init__(self):
        super().__init__("go2_skill_status_node")
        self.declare_parameter("skill_status_topic", "/go2/skill_status")
        self.declare_parameter("skill_command_topic", "/go2/skill_command")
        self.declare_parameter("goal_tolerance", 0.08)
        domain_id, interface = declare_common_dds_params(self)
        publish_hz = declare_publish_hz(self)
        self.cache = DdsStateCache()
        start_dds_subscribers(self.cache, domain_id, interface, need_box=True)
        self.tracker = SkillStatusTracker(goal_tolerance=float(self.get_parameter("goal_tolerance").value))
        self.pub = self.create_publisher(String, str(self.get_parameter("skill_status_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("skill_command_topic").value), self.on_skill_command, 10)
        self.create_timer(1.0 / publish_hz, self.publish_status)

    def on_skill_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignore invalid skill_command: {msg.data!r}")
            return
        if isinstance(payload, dict):
            self.tracker.apply_command(payload, timestamp=time.time())

    def publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.tracker.status(self.cache.snapshot()), ensure_ascii=False)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SkillStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
