#!/usr/bin/env python3
"""
Bridge node: converts raw odom-frame heights from base_height_map_node
into policy-ready heights matching the IsaacLab training formula:

    height = base_z_odom - ground_z - height_offset

Where:
    base_z_odom  = robot base Z in odom frame (from TF)
    ground_z     = raw elevation at each grid cell (odom frame)
    height_offset = 0.5 (nominal clearance, matches training)

On flat ground: 0.5 - 0.0 - 0.5 = 0.0 (centered at zero).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
import tf2_ros


class HeightmapBridgeNode(Node):

    def __init__(self):
        super().__init__("heightmap_bridge_node")

        self.declare_parameter("input_topic", "/elevation_mapping_node/base_height_map")
        self.declare_parameter("valid_topic", "/elevation_mapping_node/base_height_map_valid")
        self.declare_parameter("output_topic", "/height_sampler_node/height_map")
        self.declare_parameter("height_offset", 0.5)
        self.declare_parameter("source_frame", "odom")
        self.declare_parameter("target_frame", "base")
        self.declare_parameter("tf_timeout", 0.1)

        self.input_topic = self.get_parameter("input_topic").value
        self.valid_topic = self.get_parameter("valid_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.height_offset = float(self.get_parameter("height_offset").value)
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._valid_data = None

        self._height_sub = self.create_subscription(
            Float32MultiArray, self.input_topic, self._on_height, 10)
        self._valid_sub = self.create_subscription(
            Float32MultiArray, self.valid_topic, self._on_valid, 10)
        self._pub = self.create_publisher(Float32MultiArray, self.output_topic, 10)

        self.get_logger().info(
            f"HeightmapBridgeNode started: "
            f"input={self.input_topic}, output={self.output_topic}, "
            f"offset={self.height_offset}"
        )

    def _on_valid(self, msg: Float32MultiArray):
        self._valid_data = np.array(msg.data, dtype=np.float32)

    def _on_height(self, msg: Float32MultiArray):
        # Get base_z from TF
        try:
            transform = self._tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return

        base_z = transform.transform.translation.z

        # base_height_map_node publishes (17, 11) = (base_x, base_y) C-order
        # Policy expects (11, 17) = (base_y, base_x) C-order — transpose needed
        raw_2d = np.array(msg.data, dtype=np.float32).reshape((17, 11))
        output_2d = base_z - raw_2d - self.height_offset

        # Apply validity mask if available (on (17, 11) before transpose)
        if self._valid_data is not None and len(self._valid_data) == len(msg.data):
            valid_2d = self._valid_data.reshape((17, 11))
            output_2d[valid_2d < 0.5] = 0.0

        # Handle NaN in raw data
        output_2d[~np.isfinite(output_2d)] = 0.0

        # Transpose (17, 11) -> (11, 17) = (base_y, base_x)
        # Flip both axes to match IsaacLab convention (negative to positive)
        #   base_y: +0.5→-0.5  =>  -0.5→+0.5  (flip rows)
        #   base_x: +0.8→-0.8  =>  -0.8→+0.8  (flip cols)
        output = np.flip(output_2d.T, axis=(0, 1)).flatten()

        # Publish
        out_msg = Float32MultiArray()
        out_msg.data = output.tolist()
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeightmapBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
