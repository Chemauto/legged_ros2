#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from bishe_bridge.common import quat_wxyz_to_xyzw
from bishe_bridge.dds_state import DdsStateCache, start_dds_subscribers
from bishe_bridge.ros_params import declare_common_dds_params, declare_publish_hz


class OdomNode(Node):
    def __init__(self):
        super().__init__("go2_odom_node")
        self.declare_parameter("odom_topic", "/go2/odom")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base")
        domain_id, interface = declare_common_dds_params(self)
        publish_hz = declare_publish_hz(self)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.cache = DdsStateCache()
        start_dds_subscribers(self.cache, domain_id, interface, need_box=False)
        self.pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self.create_timer(1.0 / publish_hz, self.publish_odom)

    def publish_odom(self) -> None:
        snap = self.cache.snapshot()
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
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
