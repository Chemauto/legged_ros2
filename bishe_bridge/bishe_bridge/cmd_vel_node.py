#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from bishe_bridge.common import ControlFileWriter


class CmdVelNode(Node):
    def __init__(self):
        super().__init__("go2_cmd_vel_node")
        self.declare_parameter("cmd_vel_topic", "/go2/cmd_vel")
        self.declare_parameter("control_dir", "/tmp/mujoco_go2_control")
        self.writer = ControlFileWriter(str(self.get_parameter("control_dir").value))
        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_cmd_vel, 10)

    def on_cmd_vel(self, msg: Twist) -> None:
        text = f"{msg.linear.x} {msg.linear.y} {msg.linear.z}"
        self.writer.write_file("cmd_vel.txt", text)
        self.writer.write_file("velocity.txt", text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
