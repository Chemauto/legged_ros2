"""High-level navigation policy node.

Subscribes to robot odometry, goal pose, and heightmap.
Runs ONNX inference at 5 Hz and publishes velocity commands to the low-level controller.
"""

import math

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


def _quat_to_rot_matrix(qw, qx, qy, qz):
    """Quaternion (w,x,y,z) to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def _quat_to_yaw(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) to yaw angle."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class NavController(Node):
    def __init__(self):
        super().__init__("nav_controller_node")

        # Declare parameters
        self.declare_parameter("onnx_model_path", "")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_pose_topic", "/go2/goal_pose")
        self.declare_parameter("heightmap_topic", "/height_sampler_node/height_map")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 5.0)
        self.declare_parameter("height_scan_dim", 187)
        self.declare_parameter("action_clip_min", [-1.0, -1.0, -1.0])
        self.declare_parameter("action_clip_max", [1.0, 1.0, 1.0])

        # Read parameters
        model_path = self.get_parameter("onnx_model_path").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        goal_topic = self.get_parameter("goal_pose_topic").get_parameter_value().string_value
        hmap_topic = self.get_parameter("heightmap_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        control_hz = self.get_parameter("control_hz").get_parameter_value().double_value
        self._hmap_dim = self.get_parameter("height_scan_dim").get_parameter_value().integer_value
        self._clip_min = np.array(self.get_parameter("action_clip_min").get_parameter_value().double_array_value,
                                  dtype=np.float32)
        self._clip_max = np.array(self.get_parameter("action_clip_max").get_parameter_value().double_array_value,
                                  dtype=np.float32)

        # Load ONNX model
        self.get_logger().info(f"Loading ONNX model: {model_path}")
        self._session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        input_name = self._session.get_inputs()[0].name
        self._input_name = input_name
        self.get_logger().info(f"Model loaded. Input: {input_name}, shape: {self._session.get_inputs()[0].shape}")

        # State
        self._robot_pos = None       # [x, y, z] world frame
        self._robot_yaw = None       # float, radians
        self._gravity = None         # [3] projected gravity in body frame
        self._goal_pose = None       # [x, y, heading] world frame
        self._height_scan = None     # [hmap_dim] float32
        self._last_action = np.zeros(3, dtype=np.float32)

        # Subscribers
        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, QoSProfile(depth=10))
        self._sub_goal = self.create_subscription(
            PoseStamped, goal_topic, self._on_goal, QoSProfile(depth=10))
        self._sub_hmap = self.create_subscription(
            Float32MultiArray, hmap_topic, self._on_heightmap,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, QoSProfile(depth=1))

        # Control timer
        period = 1.0 / max(control_hz, 0.1)
        self._timer = self.create_timer(period, self._step)
        self.get_logger().info(f"Nav controller started at {control_hz} Hz")

    # ── Callbacks ──

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        self._robot_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        # projected_gravity = R_body^T * [0, 0, -1]
        R = _quat_to_rot_matrix(q.w, q.x, q.y, q.z)
        self._gravity = (R.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)

    def _on_goal(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        heading = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self._goal_pose = np.array([p.x, p.y, heading], dtype=np.float32)

    def _on_heightmap(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=np.float32)
        # Flatten and pad/truncate to expected dimension
        flat = data.flatten()
        if flat.size >= self._hmap_dim:
            self._height_scan = np.clip(flat[:self._hmap_dim], -1.0, 1.0)
        else:
            padded = np.zeros(self._hmap_dim, dtype=np.float32)
            padded[:flat.size] = flat
            self._height_scan = np.clip(padded, -1.0, 1.0)

    # ── Control loop ──

    def _step(self):
        # Check all data is available
        if any(v is None for v in [self._robot_pos, self._robot_yaw,
                                    self._gravity, self._goal_pose,
                                    self._height_scan]):
            return

        # Relative goal in world frame
        dx_w = float(self._goal_pose[0] - self._robot_pos[0])
        dy_w = float(self._goal_pose[1] - self._robot_pos[1])
        dh = float(self._goal_pose[2] - self._robot_yaw)
        dh = math.atan2(math.sin(dh), math.cos(dh))

        # Rotate to body frame: R_body^T * [dx_w, dy_w, 0]
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        dx_b = cos_yaw * dx_w + sin_yaw * dy_w
        dy_b = -sin_yaw * dx_w + cos_yaw * dy_w

        # pose_command is 4D: (dx_body, dy_body, dz_body, dh)
        # UniformPose2dCommand returns (x, y, z, heading) in body frame
        relative_goal = np.array([dx_b, dy_b, 0.0, dh], dtype=np.float32)

        # Assemble observation vector [3 + 4 + 187 + 3 = 197]
        obs = np.concatenate([
            self._gravity,          # [3]
            relative_goal,          # [4]
            self._height_scan,      # [187]
            self._last_action,      # [3]
        ]).reshape(1, -1).astype(np.float32)

        # ONNX inference
        raw_action = self._session.run(None, {self._input_name: obs})[0][0]  # [3]

        # Clip
        action = np.clip(raw_action, self._clip_min, self._clip_max)

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.linear.y = float(action[1])
        cmd.angular.z = float(action[2])
        self._cmd_pub.publish(cmd)

        # Update last action
        self._last_action = action.astype(np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = NavController()
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
