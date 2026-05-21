# nav_controller - 分层导航策略控制器

本文档说明 `nav_controller` ROS2 包的架构、配置和使用方法。该包实现了分层强化学习中的高层导航策略节点。

## 架构概览

```
/go2/goal_pose (PoseStamped)        /height_sampler_node/height_map
         |                                        |
         v                                        v
┌──────────────────────────────────────────────────────┐
│  nav_controller (Python, 5Hz)                        │
│                                                      │
│  订阅:                                               │
│    /odom           → 机器人位姿 + projected_gravity │
│    /go2/goal_pose      → 目标位姿 (世界坐标系)         │
│    /height_sampler_node/height_map → 高程图 187维      │
│                                                      │
│  处理:                                               │
│    goal_world → goal_body (yaw 旋转到机体坐标系)       │
│    组装 197 维观测向量                                  │
│    ONNX 推理 → 3D raw action [-1, 1]                  │
│                                                      │
│  输出:                                               │
│    /cmd_vel (Twist) → 速度命令                        │
└──────────────────────────┬───────────────────────────┘
                           |
                           v
┌──────────────────────────────────────────────────────┐
│  legged_rl_controller (C++, 50Hz)                    │
│                                                      │
│  订阅: /cmd_vel + /height_sampler_node/height_map    │
│  推理: walk 策略 (232维输入, 12维输出)                 │
│  输出: 关节位置目标 → 硬件/MuJoCo                      │
└──────────────────────────────────────────────────────┘
```

## 观测空间 (197 维)

| 序号 | 名称 | 维度 | 说明 |
|------|------|------|------|
| 1 | `projected_gravity` | 3 | 重力在机体坐标系的投影 (从 odom 四元数计算) |
| 2 | `pose_command` | 4 | 目标位姿 (机体坐标系下的相对偏移 dx, dy, dz, d_heading) |
| 3 | `height_scan` | 187 | 高程图 (17x11 网格, clip [-1, 1]) |
| 4 | `processed_last_action` | 3 | 上一步推理输出 (clipped) |

## 动作空间 (3 维)

| 维度 | 语义 | 范围 |
|------|------|------|
| 0 | `linear.x` (前进速度) | [-1.0, 1.0] m/s |
| 1 | `linear.y` (横向速度) | [-1.0, 1.0] m/s |
| 2 | `angular.z` (角速度) | [-1.0, 1.0] rad/s |

动作直接作为速度命令发布到 `/cmd_vel`，无需额外的 scale/offset 变换。

## 训练时的关键参数

| 参数 | 值 | 说明 |
|------|------|------|
| 高层策略频率 | 5 Hz | `decimation=40`, `physics_dt=0.005` |
| 低层策略频率 | 50 Hz | `low_level_decimation=4` |
| 高程图网格 | 17 x 11 = 187 | `resolution=0.1m`, `size=[1.6, 1.0]` |
| 高程图射线起点 | 机器人上方 20m | `ray_offset_z=20.0` |
| 高程图高度偏移 | 0.5m | `height_offset=0.5` |
| 低层 PD 增益 | Kp=25.0, Kd=0.5 | 所有关节相同 |

## 文件结构

```
nav_controller/
├── package.xml                    # ROS2 包描述
├── setup.py                       # Python 包安装
├── setup.cfg
├── nav_controller/
│   ├── __init__.py
│   └── nav_controller_node.py     # 核心节点
├── config/
│   └── nav_controller.yaml        # 参数配置
└── launch/
    └── nav_controller.launch.py   # 启动文件
```

## 使用方法

### 1. 准备 ONNX 模型

将训练好的高层导航策略导出为 ONNX 格式，放到：

```
legged_robot_description/go2_description/config/nav_policy/policy.onnx
```

导出时注意：
- 输入 shape: `[1, 197]` (batch_size=1, obs_dim=197)
- 输出 shape: `[1, 3]` (batch_size=1, action_dim=3)
- 输入 tensor 名称需与 ONNX 模型一致 (节点会自动读取)

### 2. 编译

```bash
cd /root/legged_ws
colcon build --packages-select nav_controller --symlink-install
source install/setup.bash
```

### 3. 启动

**Sim2Sim (MuJoCo 仿真):**

```bash
# 终端 1: 启动 MuJoCo 仿真
cd /home/xcj/work/Sim2Real/Mujoco
bash run_mujoco.sh

# 终端 2: 启动低层控制器 (walk 策略)
ros2 launch go2_description bringup_rl.launch.py use_rviz:=false use_rqt_cm:=false

# 终端 3: 启动高层导航控制器
ros2 launch nav_controller nav_controller.launch.py
```

**Sim2Real (真实机器人):**

```bash

  apt update && apt install -y python3-pip
  pip3 install onnxruntime


# 终端 1: 启动低层控制器
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true

# 终端 2: 启动高层导航控制器
ros2 launch nav_controller nav_controller.launch.py
```

### 4. 发送目标位姿

```bash
# 世界坐标系下的目标位姿 (x, y, heading)
ros2 topic pub --once /go2/goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 5. 自定义参数

```bash
ros2 launch nav_controller nav_controller.launch.py \
  onnx_model_path:=/path/to/your/policy.onnx \
  goal_pose_topic:=/custom/goal_topic
```

## 参数配置

配置文件: `config/nav_controller.yaml`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `onnx_model_path` | "" (由 launch 文件覆盖) | ONNX 模型路径 |
| `odom_topic` | `/odom` | 里程计 topic |
| `goal_pose_topic` | `/go2/goal_pose` | 目标位姿 topic |
| `heightmap_topic` | `/height_sampler_node/height_map` | 高程图 topic |
| `cmd_vel_topic` | `/cmd_vel` | 速度命令 topic |
| `control_hz` | 5.0 | 控制频率 |
| `height_scan_dim` | 187 | 高程图维度 |
| `action_clip_min` | [-1.0, -1.0, -1.0] | 动作下限 |
| `action_clip_max` | [1.0, 1.0, 1.0] | 动作上限 |

## 坐标系说明

- **目标位姿** (`/go2/goal_pose`): 世界坐标系下的绝对位姿
- **观测中的 pose_command**: 机体坐标系下的相对偏移 (节点自动做 world→body 变换)
- **projected_gravity**: 机体坐标系下的重力向量 (从 odom 四元数计算)

world→body 变换公式:
```
dx_body =  cos(yaw) * dx_world + sin(yaw) * dy_world
dy_body = -sin(yaw) * dx_world + cos(yaw) * dy_world
```

## 依赖

- ROS2 Humble
- `onnxruntime` (Python)
- `numpy`
- `geometry_msgs`, `nav_msgs`, `std_msgs`

## 注意事项

1. **ONNX 模型兼容性**: 确保导出的 ONNX 模型输入输出维度与训练时一致 (197 维输入, 3 维输出)
2. **高程图格式**: MuJoCo 仿真器的高程图配置 (size, resolution) 需与训练时一致
3. **低层策略**: `legged_rl_controller` 需要同时运行，使用已有的 walk 策略
4. **目标位姿坐标系**: 发送到 `/go2/goal_pose` 的位姿必须是世界坐标系下的绝对值
