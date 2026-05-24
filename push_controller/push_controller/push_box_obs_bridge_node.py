import importlib


def height_map_data_to_float_list(data, expected_size=16):
    values = [float(value) for value in data]
    if expected_size > 0 and len(values) != expected_size:
        raise ValueError(f"Expected {expected_size} values, got {len(values)}")
    return values


def _load_height_map_msg():
    msg_module = importlib.import_module("unitree_go.msg")
    return getattr(msg_module, "HeightMap")


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from std_msgs.msg import Float32MultiArray

    HeightMap = _load_height_map_msg()

    class PushBoxObsBridge(Node):
        def __init__(self):
            super().__init__("push_box_obs_bridge_node")
            self.declare_parameter("input_topic", "/push_box_obs")
            self.declare_parameter("output_topic", "/push_box_obs_float")
            self.declare_parameter("expected_size", 16)

            input_topic = (
                self.get_parameter("input_topic").get_parameter_value().string_value
            )
            output_topic = (
                self.get_parameter("output_topic").get_parameter_value().string_value
            )
            self._expected_size = (
                self.get_parameter("expected_size").get_parameter_value().integer_value
            )
            self._last_bad_size = None

            self._publisher = self.create_publisher(
                Float32MultiArray,
                output_topic,
                QoSProfile(depth=10),
            )
            self._subscription = self.create_subscription(
                HeightMap,
                input_topic,
                self._on_height_map,
                QoSProfile(depth=10),
            )
            self.get_logger().info(
                f"Bridging push-box obs {input_topic} -> {output_topic} "
                f"as Float32MultiArray[{self._expected_size}]"
            )

        def _on_height_map(self, msg):
            try:
                values = height_map_data_to_float_list(
                    msg.data,
                    expected_size=self._expected_size,
                )
            except ValueError as exc:
                bad_size = len(getattr(msg, "data", []))
                if bad_size != self._last_bad_size:
                    self.get_logger().warn(f"Ignoring push-box obs: {exc}")
                    self._last_bad_size = bad_size
                return

            out = Float32MultiArray()
            out.data = values
            self._publisher.publish(out)

    rclpy.init(args=args)
    node = PushBoxObsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
