import json

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from .geometry import (
    PUSH_OBS_DIM,
    PUSH_POLICY_OBS_DIM,
    build_push_observation,
    clip_push_action,
    is_fresh,
    projected_gravity,
    push_goal_reached,
    quat_to_yaw,
)


class PushController(Node):
    def __init__(self):
        super().__init__("push_controller_node")

        self.declare_parameter("onnx_model_path", "")
        self.declare_parameter("push_obs_topic", "/push_box_obs")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("box_pose_topic", "/push_box_pose")
        self.declare_parameter("goal_pose_topic", "/push_box_goal_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("skill_command_topic", "/go2/skill_command")
        self.declare_parameter("control_hz", 5.0)
        self.declare_parameter("use_external_push_obs", True)
        self.declare_parameter("push_obs_timeout_sec", 0.2)
        self.declare_parameter("stop_on_goal", True)
        self.declare_parameter("goal_tolerance_xy", 0.08)
        self.declare_parameter("enabled_on_start", True)

        model_path = self.get_parameter("onnx_model_path").get_parameter_value().string_value
        push_obs_topic = self.get_parameter("push_obs_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        box_pose_topic = self.get_parameter("box_pose_topic").get_parameter_value().string_value
        goal_pose_topic = self.get_parameter("goal_pose_topic").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        skill_command_topic = (
            self.get_parameter("skill_command_topic").get_parameter_value().string_value
        )
        control_hz = self.get_parameter("control_hz").get_parameter_value().double_value
        self._use_external_push_obs = (
            self.get_parameter("use_external_push_obs").get_parameter_value().bool_value
        )
        self._push_obs_timeout_sec = (
            self.get_parameter("push_obs_timeout_sec").get_parameter_value().double_value
        )
        self._stop_on_goal = self.get_parameter("stop_on_goal").get_parameter_value().bool_value
        self._goal_tolerance_xy = (
            self.get_parameter("goal_tolerance_xy").get_parameter_value().double_value
        )
        self._enabled = self.get_parameter("enabled_on_start").get_parameter_value().bool_value
        self._goal_reached = False

        self.get_logger().info(f"Loading push-box ONNX model: {model_path}")
        self._session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self._input_name = self._session.get_inputs()[0].name
        self.get_logger().info(
            f"Push-box model loaded. Input: {self._input_name}, "
            f"shape: {self._session.get_inputs()[0].shape}"
        )

        self._external_push_obs = None
        self._external_push_obs_stamp_sec = None
        self._robot_position = None
        self._robot_yaw = None
        self._projected_gravity = None
        self._base_ang_vel = None
        self._box_position = None
        self._box_yaw = None
        self._goal_position = None
        self._goal_yaw = None
        self._last_action = np.zeros(3, dtype=np.float32)

        self._sub_push_obs = self.create_subscription(
            Float32MultiArray,
            push_obs_topic,
            self._on_push_obs,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, QoSProfile(depth=10))
        self._sub_box_pose = self.create_subscription(
            PoseStamped, box_pose_topic, self._on_box_pose, QoSProfile(depth=10))
        self._sub_goal_pose = self.create_subscription(
            PoseStamped, goal_pose_topic, self._on_goal_pose, QoSProfile(depth=10))
        self._sub_skill_command = self.create_subscription(
            String, skill_command_topic, self._on_skill_command, QoSProfile(depth=10))
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, QoSProfile(depth=1))

        period = 1.0 / max(control_hz, 0.1)
        self._timer = self.create_timer(period, self._step)
        self.get_logger().info(f"Push controller started at {control_hz} Hz")

    def _on_skill_command(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return

        model_use = payload.get("model_use")
        skill = str(payload.get("skill") or "").strip().lower()
        start = payload.get("start")

        if model_use == 3 or skill in {"push", "push_box"}:
            self._enabled = start is not False
            if self._enabled:
                self._goal_reached = False
            else:
                self._publish_zero()
            return

        if start is False or model_use not in (None, 3) or skill in {"idle", "stop", "off"}:
            self._enabled = False
            self._goal_reached = False
            self._publish_zero()

    def _on_push_obs(self, msg):
        data = np.array(msg.data, dtype=np.float32).flatten()
        if data.size != PUSH_OBS_DIM:
            self.get_logger().warn(
                f"Ignoring push-box observation with dim {data.size}; expected {PUSH_OBS_DIM}."
            )
            return
        self._external_push_obs = data
        self._external_push_obs_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        w = msg.twist.twist.angular
        self._robot_position = np.array([p.x, p.y, p.z], dtype=np.float64)
        self._robot_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._projected_gravity = projected_gravity(q.x, q.y, q.z, q.w)
        self._base_ang_vel = np.array([w.x, w.y, w.z], dtype=np.float32)

    def _on_box_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self._box_position = np.array([p.x, p.y, p.z], dtype=np.float64)
        self._box_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _on_goal_pose(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self._goal_position = np.array([p.x, p.y, p.z], dtype=np.float64)
        self._goal_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._enabled = True
        self._goal_reached = False

    def _step(self):
        if not self._enabled:
            self._publish_zero()
            return

        obs = self._get_push_obs()
        if obs is None:
            self._publish_zero()
            return

        if self._stop_on_goal and push_goal_reached(obs, self._goal_tolerance_xy):
            if not self._goal_reached:
                self.get_logger().info(
                    f"Push goal reached within {self._goal_tolerance_xy:.3f} m; publishing zero velocity."
                )
            self._goal_reached = True
            self._publish_zero()
            return
        self._goal_reached = False

        policy_obs = np.concatenate([obs, self._last_action]).reshape(1, -1).astype(np.float32)
        if policy_obs.shape[1] != PUSH_POLICY_OBS_DIM:
            self.get_logger().error(
                f"Push policy observation dim mismatch: {policy_obs.shape[1]} != {PUSH_POLICY_OBS_DIM}"
            )
            return

        raw_action = self._session.run(None, {self._input_name: policy_obs})[0][0]
        action = clip_push_action(raw_action)

        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.linear.y = float(action[1])
        cmd.angular.z = float(action[2])
        self._cmd_pub.publish(cmd)
        self._last_action = action

    def _publish_zero(self):
        cmd = Twist()
        self._cmd_pub.publish(cmd)
        self._last_action = np.zeros(3, dtype=np.float32)

    def _get_push_obs(self):
        if (
            self._use_external_push_obs
            and self._external_push_obs is not None
            and self._external_push_obs_stamp_sec is not None
            and is_fresh(
                self.get_clock().now().nanoseconds * 1e-9,
                self._external_push_obs_stamp_sec,
                self._push_obs_timeout_sec,
            )
        ):
            return self._external_push_obs

        required = [
            self._robot_position,
            self._robot_yaw,
            self._projected_gravity,
            self._base_ang_vel,
            self._box_position,
            self._box_yaw,
            self._goal_position,
            self._goal_yaw,
        ]
        if any(value is None for value in required):
            return None

        return build_push_observation(
            robot_position=self._robot_position,
            robot_yaw=self._robot_yaw,
            projected_gravity=self._projected_gravity,
            base_ang_vel=self._base_ang_vel,
            box_position=self._box_position,
            box_yaw=self._box_yaw,
            goal_position=self._goal_position,
            goal_yaw=self._goal_yaw,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PushController()
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
