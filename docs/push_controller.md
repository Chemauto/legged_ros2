# push_controller - 分层推箱子策略控制器

本文档说明 `push_controller` ROS 2 包的架构、配置和使用方法。该包实现推箱子任务的高层策略节点，高层输出速度命令，低层由 `legged_rl_controller` 执行 walking policy。

## 架构概览

```
/push_box_obs (Float32MultiArray, 16D)
         |
         v
┌──────────────────────────────────────────────────────┐
│  push_controller (Python, 5Hz)                       │
│                                                      │
│  优先订阅:                                           │
│    /push_box_obs → 16 维推箱子观测                    │
│                                                      │
│  fallback 订阅:                                      │
│    /odom                 → 机器人位姿、角速度、重力方向 │
│    /push_box_pose        → 箱子当前位姿                │
│    /push_box_goal_pose   → 箱子目标位姿                │
│                                                      │
│  推理:                                               │
│    push_policy/policy.onnx → 3D velocity action       │
│                                                      │
│  输出:                                               │
│    /cmd_vel (Twist)                                  │
└──────────────────────────┬───────────────────────────┘
                           |
                           v
┌──────────────────────────────────────────────────────┐
│  legged_rl_controller (C++, 50Hz)                    │
│                                                      │
│  订阅: /cmd_vel + /height_sampler_node/height_map    │
│  策略: push_policy/low_level_policy                  │
│  输出: 关节位置目标 → 硬件/MuJoCo                      │
└──────────────────────────────────────────────────────┘
```

## 策略文件

高层策略：

```text
legged_robot_description/go2_description/config/push_policy/policy.onnx
legged_robot_description/go2_description/config/push_policy/IO_descriptors.yaml
```

低层策略：

```text
legged_robot_description/go2_description/config/push_policy/low_level_policy/policy.onnx
legged_robot_description/go2_description/config/push_policy/low_level_policy/IO_descriptors.yaml
```

## 观测与动作

高层策略输入为 19 维：

| 部分 | 维度 | 说明 |
|------|------|------|
| `base_ang_vel` | 3 | 机器人机体系角速度 |
| `projected_gravity` | 3 | 重力在机器人机体系下的投影 |
| `box_in_robot_frame_pos` | 3 | 箱子在机器人坐标系下的位置 |
| `box_in_robot_frame_yaw` | 2 | 箱子相对机器人 yaw，sin/cos 编码 |
| `goal_in_box_frame_pos` | 3 | 目标在箱子坐标系下的位置 |
| `goal_in_box_frame_yaw` | 2 | 目标相对箱子 yaw，sin/cos 编码 |
| `processed_last_action` | 3 | 上一次裁剪后的高层动作 |

如果使用 `/push_box_obs`，该话题只需要发布前 16 维；`push_controller` 会自动追加 3 维 `last_action`。

动作输出为 3 维 `/cmd_vel`：

| 维度 | 语义 | 裁剪范围 |
|------|------|----------|
| 0 | `linear.x` | [-0.5, 1.0] |
| 1 | `linear.y` | [-1.0, 1.0] |
| 2 | `angular.z` | [-0.5, 0.5] |

## 编译

```bash
cd /root/legged_ws
colcon build --packages-select go2_description push_controller --symlink-install
source install/setup.bash
```

## 启动

推荐使用一键 launch：

```bash
ros2 launch go2_description bringup_push.launch.py \
  use_rviz:=false \
  use_rqt_cm:=false
```

`bringup_push.launch.py` 内部会做两件事：

- include `go2_description/launch/bringup_rl.launch.py`，并传入 `policy_profile:=push_low_level`
- include `push_controller/launch/push_controller.launch.py`，并加载高层模型 `go2_description/config/push_policy/policy.onnx`

## 话题要求

推荐最小话题：

```text
/odom
/height_sampler_node/height_map
/push_box_obs
/cmd_vel
```

其中 `/push_box_obs` 为：

```text
std_msgs/msg/Float32MultiArray
```

长度必须是 16。该话题超过 `push_obs_timeout_sec` 后会被视为失效，默认超时为 0.2 秒。

如果没有 `/push_box_obs`，则需要 fallback 话题：

```text
/odom
/height_sampler_node/height_map
/push_box_pose
/push_box_goal_pose
/cmd_vel
```

`/push_box_pose` 和 `/push_box_goal_pose` 类型均为：

```text
geometry_msgs/msg/PoseStamped
```

## 自定义参数

```bash
ros2 launch go2_description bringup_push.launch.py \
  high_level_onnx_model_path:=/path/to/push_policy.onnx \
  push_obs_topic:=/custom_push_box_obs \
  goal_pose_topic:=/custom_push_box_goal_pose \
  use_rviz:=false \
  use_rqt_cm:=false
```

如果只想单独启动高层节点：

```bash
ros2 launch push_controller push_controller.launch.py \
  onnx_model_path:=/path/to/push_policy.onnx \
  push_obs_topic:=/custom_push_box_obs
```

## 联调检查

1. 确认低层已激活：

```bash
ros2 control list_controllers
```

应看到：

```text
rl_controller active
```

2. 确认高层正在发布速度：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
```

应看到 `/cmd_vel` 的 publisher 中包含 `push_controller_node`。

3. 确认推箱子观测存在：

```bash
ros2 topic hz /push_box_obs
ros2 topic echo --once /push_box_obs
```

如果没有 `/push_box_obs`，确认 fallback 话题：

```bash
ros2 topic hz /push_box_pose
ros2 topic hz /push_box_goal_pose
```

## 常见问题

### 发目标后没有反应

先看 `/cmd_vel` 是否有 `push_controller_node` 发布者。如果没有，通常是 `push_controller_node` 没启动、ONNX 加载失败、缺少 `onnxruntime`，或缺少推箱子观测。

### `/push_box_obs` 有一帧后停止

节点不会一直复用旧观测。超过 `push_obs_timeout_sec` 后，旧 `/push_box_obs` 会失效。需要让观测源持续发布，或启用 fallback 话题。

### 容器内缺少 onnxruntime

当前 Dockerfile 已安装 Python 版 `onnxruntime==1.22.0`。重新 build 镜像后验证：

```bash
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
```
