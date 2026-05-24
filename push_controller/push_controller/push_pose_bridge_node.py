from types import SimpleNamespace


def _copy_position(position):
    return SimpleNamespace(
        x=float(getattr(position, "x", 0.0)),
        y=float(getattr(position, "y", 0.0)),
        z=float(getattr(position, "z", 0.0)),
    )


def _copy_orientation(orientation):
    return SimpleNamespace(
        x=float(getattr(orientation, "x", 0.0)),
        y=float(getattr(orientation, "y", 0.0)),
        z=float(getattr(orientation, "z", 0.0)),
        w=float(getattr(orientation, "w", 1.0)),
    )


def _zero_twist():
    return SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
    )


def pose_to_odometry(pose_msg, child_frame_id="base", odometry_cls=None):
    if odometry_cls is None:
        try:
            from nav_msgs.msg import Odometry

            odometry_cls = Odometry
        except Exception:
            odometry_cls = None

    odom = odometry_cls() if odometry_cls is not None else _empty_odometry()
    odom.header.frame_id = getattr(pose_msg.header, "frame_id", "")
    odom.header.stamp = getattr(pose_msg.header, "stamp", None)
    odom.child_frame_id = child_frame_id
    position = _copy_position(pose_msg.pose.position)
    orientation = _copy_orientation(pose_msg.pose.orientation)
    odom.pose.pose.position.x = position.x
    odom.pose.pose.position.y = position.y
    odom.pose.pose.position.z = position.z
    odom.pose.pose.orientation.x = orientation.x
    odom.pose.pose.orientation.y = orientation.y
    odom.pose.pose.orientation.z = orientation.z
    odom.pose.pose.orientation.w = orientation.w
    odom.twist.twist.linear.x = 0.0
    odom.twist.twist.linear.y = 0.0
    odom.twist.twist.linear.z = 0.0
    odom.twist.twist.angular.x = 0.0
    odom.twist.twist.angular.y = 0.0
    odom.twist.twist.angular.z = 0.0
    return odom


def _empty_odometry():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="", stamp=None),
        child_frame_id="",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(twist=_zero_twist()),
    )


def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    class PushPoseBridge(Node):
        def __init__(self):
            super().__init__("push_pose_bridge_node")
            self.declare_parameter("robot_pose_topic", "/mocap/unitree_go2/pose")
            self.declare_parameter("box_pose_input_topic", "/mocap/unitree_box/pose")
            self.declare_parameter("odom_topic", "/Odometry")
            self.declare_parameter("push_box_pose_topic", "/push_box_pose")
            self.declare_parameter("child_frame_id", "base")

            robot_pose_topic = (
                self.get_parameter("robot_pose_topic").get_parameter_value().string_value
            )
            box_pose_input_topic = (
                self.get_parameter("box_pose_input_topic").get_parameter_value().string_value
            )
            odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
            push_box_pose_topic = (
                self.get_parameter("push_box_pose_topic").get_parameter_value().string_value
            )
            self._child_frame_id = (
                self.get_parameter("child_frame_id").get_parameter_value().string_value
            )

            qos = QoSProfile(depth=10)
            self._odom_pub = self.create_publisher(Odometry, odom_topic, qos)
            self._box_pose_pub = self.create_publisher(PoseStamped, push_box_pose_topic, qos)
            self._robot_pose_sub = self.create_subscription(
                PoseStamped,
                robot_pose_topic,
                self._on_robot_pose,
                qos,
            )
            self._box_pose_sub = self.create_subscription(
                PoseStamped,
                box_pose_input_topic,
                self._on_box_pose,
                qos,
            )

            self.get_logger().info(
                f"Bridging robot pose {robot_pose_topic} -> {odom_topic}; "
                f"box pose {box_pose_input_topic} -> {push_box_pose_topic}"
            )

        def _on_robot_pose(self, msg):
            self._odom_pub.publish(
                pose_to_odometry(msg, child_frame_id=self._child_frame_id, odometry_cls=Odometry)
            )

        def _on_box_pose(self, msg):
            self._box_pose_pub.publish(msg)

    rclpy.init(args=args)
    node = PushPoseBridge()
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
