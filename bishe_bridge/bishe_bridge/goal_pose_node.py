#!/usr/bin/env python3
from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from bishe_bridge.common import ControlFileWriter


class GoalPoseNode(Node):
    def __init__(self):
        super().__init__("go2_goal_pose_node")
        self.declare_parameter("goal_pose_topic", "/go2/goal_pose")
        self.declare_parameter("control_dir", "/tmp/mujoco_go2_control")
        self.writer = ControlFileWriter(str(self.get_parameter("control_dir").value))
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_pose_topic").value), self.on_goal_pose, 10)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        goal = [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)]
        self.writer.write_file("goal.txt", f"{goal[0]} {goal[1]} {goal[2]}")
        self.writer.write_file("goal_pose.json", json.dumps({"goal": goal}, ensure_ascii=False))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
