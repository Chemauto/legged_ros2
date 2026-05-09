#!/usr/bin/env python3
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from bishe_bridge.common import build_scene_objects, parse_scene_objects_config
from bishe_bridge.dds_state import DdsStateCache, start_dds_subscribers
from bishe_bridge.ros_params import declare_common_dds_params, declare_publish_hz


class SceneObjectsNode(Node):
    def __init__(self):
        super().__init__("go2_scene_objects_node")
        self.declare_parameter("scene_objects_topic", "/go2/scene_objects")
        self.declare_parameter("static_scene_objects", "")
        domain_id, interface = declare_common_dds_params(self)
        publish_hz = declare_publish_hz(self)
        self.static_objects = parse_scene_objects_config(str(self.get_parameter("static_scene_objects").value))
        self.cache = DdsStateCache()
        start_dds_subscribers(self.cache, domain_id, interface, need_box=True)
        self.pub = self.create_publisher(String, str(self.get_parameter("scene_objects_topic").value), 10)
        self.create_timer(1.0 / publish_hz, self.publish_scene_objects)

    def publish_scene_objects(self) -> None:
        msg = String()
        msg.data = json.dumps(build_scene_objects(self.cache.snapshot(), self.static_objects), ensure_ascii=False)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneObjectsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
