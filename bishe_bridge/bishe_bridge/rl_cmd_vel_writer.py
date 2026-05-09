#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class RlCmdVelWriter(Node):
    def __init__(self, control_dir: str, topic: str, publish_hz: float):
        super().__init__("rl_cmd_vel_writer")
        self.velocity_file = Path(control_dir).expanduser() / "velocity.txt"
        self.pub = self.create_publisher(Twist, topic, 10)
        self.create_timer(1.0 / max(float(publish_hz), 0.1), self.publish_velocity)

    def publish_velocity(self) -> None:
        velocity = self.read_velocity()
        msg = Twist()
        msg.linear.x = velocity[0]
        msg.linear.y = velocity[1]
        msg.linear.z = velocity[2]
        self.pub.publish(msg)

    def read_velocity(self) -> tuple[float, float, float]:
        try:
            parts = self.velocity_file.read_text(encoding="utf-8").strip().split()
            if len(parts) >= 3:
                return float(parts[0]), float(parts[1]), float(parts[2])
        except (FileNotFoundError, OSError, ValueError):
            pass
        return 0.0, 0.0, 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish RL cmd_vel from bishe_bridge control files.")
    parser.add_argument("--control-dir", default="/tmp/mujoco_go2_control")
    parser.add_argument("--topic", default="rl_cmd_vel")
    parser.add_argument("--publish-hz", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = RlCmdVelWriter(args.control_dir, args.topic, args.publish_hz)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
