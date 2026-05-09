#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from bishe_bridge.common import box_world_position_from_snapshot
from bishe_bridge.dds_state import DdsStateCache, start_dds_subscribers
from bishe_bridge.ros_params import declare_common_dds_params, declare_publish_hz


class BoxPoseNode(Node):
    def __init__(self):
        super().__init__("go2_box_pose_node")
        self.declare_parameter("box_pose_topic", "/go2/box_pose")
        self.declare_parameter("frame_id", "map")
        domain_id, interface = declare_common_dds_params(self)
        publish_hz = declare_publish_hz(self)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.cache = DdsStateCache()
        start_dds_subscribers(self.cache, domain_id, interface, need_box=True)
        self.pub = self.create_publisher(PoseStamped, str(self.get_parameter("box_pose_topic").value), 10)
        self.create_timer(1.0 / publish_hz, self.publish_box_pose)

    def publish_box_pose(self) -> None:
        box_pos = box_world_position_from_snapshot(self.cache.snapshot())
        if box_pos is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = box_pos[0]
        msg.pose.position.y = box_pos[1]
        msg.pose.position.z = box_pos[2]
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
