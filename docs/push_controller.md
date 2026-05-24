# push_controller - 分层推箱子策略控制器

本文档说明 `push_controller` ROS 2 包的架构、配置和使用方法。该包实现推箱子任务的高层策略节点，高层输出速度命令，低层由 `legged_rl_controller` 执行 walking policy。

## 架构概览

```
/push_box_obs (unitree_go/HeightMap, 16D, MuJoCo/Unitree DDS)
         |
         v
┌──────────────────────────────────────────────────────┐
│  push_box_obs_bridge_node                            │
│  输出: /push_box_obs_float (Float32MultiArray, 16D)  │
└──────────────────────────┬───────────────────────────┘
                           |
                           v
┌──────────────────────────────────────────────────────┐
│  push_controller (Python, 5Hz)                       │
│                                                      │
│  优先订阅:                                           │
│    /push_box_obs_float → 16 维推箱子观测              │
│                                                      │
│  fallback 订阅:                                      │
│    /Odometry             → 机器人位姿、角速度、重力方向 │
│    /push_box_pose        → 箱子当前位姿                │
│    /push_box_goal_pose   → 箱子目标位姿                │
│                                                      │
│  推理:                                               │
│    push_policy/policy.onnx → 3D velocity action       │
│                                                      │
│  输出:                                               │
│    /push_cmd_vel (Twist)                             │
└──────────────────────────┬───────────────────────────┘
                           |
                           v
┌──────────────────────────────────────────────────────┐
│  legged_rl_controller (C++, 50Hz)                    │
│                                                      │
│  订阅: /push_cmd_vel + /height_sampler_node/height_map│
│  策略: push_policy/low_level_policy                  │
│  输出: 关节位置目标 → 硬件/MuJoCo                      │
└──────────────────────────────────────────────────────┘
```

MuJoCo 仿真时，推箱目标还有一条单独的同步链路：

```text
/go2/skill_command (model_use:3 + goal)
        或
/push_box_goal_pose
        或
/go2/goal_pose（仅当最近一次 skill_command 是 model_use:3）
        |
        v
Mujoco/simulate_python/mujoco_ros2_bridge.py
        |
        v
/tmp/mujoco_go2_control/push_box_goal.txt
        |
        v
PushBoxSdk2Bridge 读取目标后发布 rt/push_box_obs
```

MuJoCo 内部不再保留 `(1.7, 0.0, 0.12)` 这种默认推箱目标。没有外部目标时，MuJoCo 不发布带目标的 `rt/push_box_obs`，避免一启动就开始推箱。`mujoco_ros2_bridge.py` 启动时会自动清理旧的 `push_box_goal.txt`，收到 stop/idle/非 push 技能时也会清理推箱目标。

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

如果使用外部观测，该话题只需要提供前 16 维；`push_controller` 会自动追加 3 维 `last_action`。在 `bringup_push.launch.py` 中，MuJoCo/Unitree 原始 `/push_box_obs` 是 `unitree_go/msg/HeightMap`，会先由 `push_box_obs_bridge_node` 转成 `/push_box_obs_float`。

动作输出为 3 维 `/push_cmd_vel`：

| 维度 | 语义 | 裁剪范围 |
|------|------|----------|
| 0 | `linear.x` | [-0.5, 1.0] |
| 1 | `linear.y` | [-1.0, 1.0] |
| 2 | `angular.z` | [-0.5, 0.5] |

默认裁剪范围在 `push_controller/config/push_controller.yaml` 中配置：

```yaml
action_clip_min: [-0.5, -1.0, -0.5]
action_clip_max: [1.0, 1.0, 0.5]
```

实机如果需要更保守的推箱速度，可以改成：

```yaml
action_clip_min: [-0.5, -0.5, -0.5]
action_clip_max: [0.5, 0.5, 0.5]
```

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

如果启动后默认还是 `stand_static_controller` active，需要切到推箱低层控制器：

```bash
ros2 control switch_controllers \
  --deactivate stand_static_controller \
  --activate rl_controller \
  --strict
```

`bringup_push.launch.py` 内部会做三件事：

- include `go2_description/launch/bringup_rl.launch.py`，并传入 `policy_profile:=push_low_level`
- 启动 `push_box_obs_bridge_node`，把原始 `/push_box_obs` 转成 `/push_box_obs_float`
- include `push_controller/launch/push_controller.launch.py`，加载高层模型 `go2_description/config/push_policy/policy.onnx`，并让高层输出 `/push_cmd_vel`

默认 `enabled_on_start:=false`。重启 `bringup_push.launch.py` 后，`push_controller` 不应在收到新的 push 命令或 `/push_box_goal_pose` 前主动执行旧任务。

如果是在 MuJoCo 仿真中测试，需要同时重启 `/home/xcj/work/Sim2Real/Mujoco/run_mujoco.sh`，因为推箱目标桥接在 MuJoCo 进程里：

```bash
cd /home/xcj/work/Sim2Real/Mujoco
bash run_mujoco.sh
```

## 话题要求

推荐最小话题：

```text
/Odometry
/height_sampler_node/height_map
/push_box_obs
/push_box_obs_float
/push_cmd_vel
```

其中 `/push_box_obs` 来自 MuJoCo/Unitree DDS：

```text
unitree_go/msg/HeightMap
```

`bringup_push.launch.py` 会默认桥接成 `/push_box_obs_float`：

```text
std_msgs/msg/Float32MultiArray
```

长度必须是 16。该话题超过 `push_obs_timeout_sec` 后会被视为失效，默认超时为 0.2 秒。

如果没有 `/push_box_obs`，则需要 fallback 话题：

```text
/Odometry
/height_sampler_node/height_map
/push_box_pose
/push_box_goal_pose
/push_cmd_vel
```

`/push_box_pose` 和 `/push_box_goal_pose` 类型均为：

```text
geometry_msgs/msg/PoseStamped
```

## 发送推箱目标

### MuJoCo 推荐命令

如果是 MuJoCo 仿真，推荐直接用 `/go2/skill_command` 发送推箱技能和目标点。目标数组为：

```text
[x, y, z, yaw]
```

例如目标在世界坐标系前方 1.5m：

```bash
ros2 topic pub --once /go2/skill_command std_msgs/String \
  "{data: '{\"model_use\": 3, \"skill\": \"push\", \"goal\": [1.5, 0.0, 0.0, 0.0], \"start\": true}'}"
```

这条命令会做两件事：

- `push_controller_node` 收到 push 技能后使能推箱输出
- MuJoCo 的 `mujoco_ros2_bridge.py` 将目标写入 `/tmp/mujoco_go2_control/push_box_goal.txt`，随后 `PushBoxSdk2Bridge` 开始发布带目标的 `rt/push_box_obs`

也可以继续直接发 ROS fallback 目标：

```bash
ros2 topic pub --once /push_box_goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

如果只发 `/go2/goal_pose`，需要先让 bridge 知道当前是 push 技能：

```bash
ros2 topic pub --once /go2/skill_command std_msgs/String \
  "{data: '{\"model_use\": 3, \"skill\": \"push\", \"start\": true}'}"

ros2 topic pub --once /go2/goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

检查目标是否已经同步到 MuJoCo：

```bash
cat /tmp/mujoco_go2_control/push_box_goal.txt
ros2 topic hz /push_box_obs_float
ros2 topic hz /push_cmd_vel
```

### 实机或 fallback 方式

`push_controller` 需要“箱子当前位姿”和“箱子目标位姿”来推理速度：

```text
/push_box_pose       当前箱子位姿
/push_box_goal_pose  目标箱子位姿
```

如果已经有 MuJoCo/Unitree 持续发布原始 `/push_box_obs`，则不需要手动发布下面两个 fallback 话题；桥接节点会生成 `/push_box_obs_float` 给高层策略使用。

如果没有原始 `/push_box_obs`，可以用 fallback 方式测试。先发布箱子当前位姿：

```bash
ros2 topic pub -r 10 /push_box_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 0.6, y: 0.0, z: 0.1}, orientation: {w: 1.0}}}"
```

再发布箱子目标位姿。当前 demo 约定所有目标点在机器人前方 1.5m：

```bash
ros2 topic pub --once /push_box_goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

实机部署时 `/push_box_pose` 不能一直用手动静态值，否则箱子被推动后观测会变旧，策略会按错误箱子位置继续输出。实机需要由感知节点持续发布真实箱子位姿，或者持续发布 16 维 `/push_box_obs` 原始观测并由桥接节点转成 `/push_box_obs_float`。

如果通过 FQPlanner/`llmservice` 发送技能命令，则使用：

```bash
cd /home/xcj/work/Sim2Real/FQPlanner
export ROBOT_WS_URL=ws://<robot_deploy_ip>:8765
python demos/run_demo.py push_climb
```

其中 `push_climb` 会先发送 `push(x=1.5, y=0.0, yaw=0.0)`，再发送 `climb(height=0.5)`。

## 停止推箱

### 自动停止

`push_controller` 会根据观测中的 `goal_in_box_frame_pos` 判断箱子是否到达目标。默认 `goal_tolerance_xy=0.12`，也就是箱子目标在箱子坐标系下的 x/y 距离小于 12cm 时，高层停止继续推理并持续向 `/push_cmd_vel` 发布零速度。

检查是否已经停止输出运动速度：

```bash
ros2 topic echo /push_cmd_vel
```

应看到：

```text
linear:
  x: 0.0
  y: 0.0
angular:
  z: 0.0
```

### 手动安全停止

实机上最可靠的停止方式是切回站立控制器：

```bash
ros2 control switch_controllers \
  --deactivate rl_controller \
  --activate stand_static_controller \
  --strict
```

如果只是想让速度归零，但不切控制器，可以临时发布一次零速到推箱速度话题：

```bash
ros2 topic pub --once /push_cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

注意：如果 `push_controller_node` 仍在运行且观测仍有效，它下一帧可能会继续发布策略速度。因此实机安全停止优先使用 controller 切换。

### 通过技能命令停止

`push_controller_node` 订阅 `/go2/skill_command`。收到非 push 技能、`start:false`、`idle`、`stop` 或 `off` 时，会禁用推箱输出并发布零速度。可以手动发送：

```bash
ros2 topic pub --once /go2/skill_command std_msgs/String \
  "{data: '{\"model_use\": 0, \"skill\": \"stop\", \"start\": false}'}"
```

随后再确认：

```bash
ros2 topic echo --once /push_cmd_vel
```

## 自定义参数

```bash
ros2 launch go2_description bringup_push.launch.py \
  high_level_onnx_model_path:=/path/to/push_policy.onnx \
  raw_push_obs_topic:=/push_box_obs \
  converted_push_obs_topic:=/push_box_obs_float \
  push_obs_topic:=/push_box_obs_float \
  odom_topic:=/Odometry \
  cmd_vel_topic:=/push_cmd_vel \
  goal_tolerance_xy:=0.12 \
  enabled_on_start:=false \
  goal_pose_topic:=/custom_push_box_goal_pose \
  use_rviz:=false \
  use_rqt_cm:=false
```

`converted_push_obs_topic` 是桥接节点输出，`push_obs_topic` 是高层策略输入；如果自定义其中一个，另一个也要保持一致。

如果只想单独启动高层节点：

```bash
ros2 launch push_controller push_controller.launch.py \
  onnx_model_path:=/path/to/push_policy.onnx \
  push_obs_topic:=/custom_push_box_obs \
  cmd_vel_topic:=/cmd_vel
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
ros2 topic info /push_cmd_vel -v
ros2 topic echo /push_cmd_vel
```

应看到 `/push_cmd_vel` 的 publisher 中包含 `push_controller_node`，并且 `rl_controller` 是 subscriber。

3. 确认推箱子观测存在：

```bash
ros2 topic hz /push_box_obs
ros2 topic hz /push_box_obs_float
ros2 topic echo --once /push_box_obs_float
```

如果没有 `/push_box_obs`，确认 fallback 话题：

```bash
ros2 topic hz /push_box_pose
ros2 topic hz /push_box_goal_pose
```

MuJoCo 仿真还需要确认目标桥接文件已经生成：

```bash
cat /tmp/mujoco_go2_control/push_box_goal.txt
```

如果文件不存在，说明还没有发送 `/go2/skill_command` 的 push goal，或者没有发送 `/push_box_goal_pose`。

## 真实部署注意事项

### 推箱策略部署

1. 不要同时启动 `bringup_nav.launch.py` 和 `bringup_push.launch.py`。这两个 launch 都会启动底层 `bringup_rl.launch.py` 和同一个 `rl_controller`，同时启动会重复创建控制节点和 controller manager。需要导航时启动导航 bringup，需要推箱时启动推箱 bringup，或者由上层任务管理器负责先停一个再启另一个。

2. 推箱速度已经隔离到 `/push_cmd_vel`，不要让推箱高层直接发布 `/cmd_vel`。实际检查时应看到：

```bash
ros2 topic info /push_cmd_vel -v
```

其中 publisher 是 `push_controller_node`，subscriber 是 `rl_controller`。如果 `rl_controller` 仍订阅 `/cmd_vel`，说明启动的不是 `bringup_push.launch.py`，或者 launch 参数没有传到 `bringup_rl.launch.py`。

3. MuJoCo/Unitree 原始 `/push_box_obs` 是 `unitree_go/msg/HeightMap`，不是 `std_msgs/msg/Float32MultiArray`。真实部署时不要再手动向 `/push_box_obs` 发布 `Float32MultiArray`，否则会重新出现同名多类型冲突。高层策略应只订阅 `/push_box_obs_float`。

4. `/push_box_obs_float` 必须持续发布 16 维观测。`push_controller` 默认 0.2 秒超时，观测断流后不会继续复用旧值。实机如果没有直接的 16 维推箱观测，就必须持续提供 `/push_box_pose` 和 `/push_box_goal_pose` fallback 数据。

5. `/push_box_pose`、`/push_box_goal_pose` 和 `/Odometry` 必须在同一世界坐标系下。代码当前不读取 `header.frame_id` 做 TF 变换，只直接使用数值，所以 `map`、`odom` 名字本身不重要，重要的是数值必须一致。

6. MuJoCo 仿真下，`/push_box_goal_pose` 和 `/go2/skill_command` 目标会同步到 `/tmp/mujoco_go2_control/push_box_goal.txt`。实机部署不依赖这个文件，实机只需要 ROS/DDS 观测链路正确。

7. 高层策略会在 `goal_tolerance_xy` 范围内发布零速度，但不会自动切回站立控制器。任务完成后，上层仍应停止发送推箱任务、切回静态控制器，或让 `llmservice`/任务管理器发送下一步技能。不要在实机上长期让旧目标和旧箱子位姿继续驱动策略。

8. Docker 内需要提前具备 `onnxruntime` 和 `unitree_go` 消息包。启动前至少验证：

```bash
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
ros2 interface show unitree_go/msg/HeightMap
```

### 导航策略部署

1. 导航速度已经隔离到 `/nav_cmd_vel`。真实部署时应检查：

```bash
ros2 topic info /nav_cmd_vel -v
```

publisher 应为 `nav_controller_node`，subscriber 应为 `rl_controller`。遥控器仍可能发布 `/cmd_vel`，但导航 bringup 下的 `rl_controller` 不应订阅 `/cmd_vel`。

2. 当前实机验证版导航里程计话题是 `/Odometry`。`bringup_nav.launch.py` 已默认使用 `/Odometry`，MuJoCo 仿真时通常需要显式覆盖为 `odom_topic:=/odom`。

3. 导航目标 `/go2/goal_pose` 是世界坐标系下的绝对目标，代码同样不使用 TF 转换 `header.frame_id`。因此目标坐标必须和 `odom_topic` 的坐标系一致。发送正前方 1.5m 的常用测试命令是：

```bash
ros2 topic pub --once /go2/goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

4. 导航高层依赖 `/Odometry` 和 `/height_sampler_node/height_map`。高程图必须是 187 维，且与训练时的高度图范围、分辨率和安装坐标一致。实机部署前检查：

```bash
ros2 topic hz /Odometry
ros2 topic hz /height_sampler_node/height_map
ros2 topic echo --once /height_sampler_node/height_map
```

5. 如果使用 mapping/TF 修正链路，RViz 固定坐标建议使用 `odom`，并确认 `odom -> initial_base/camera_init/body/base` 这条链稳定。导航策略本身只吃 `odom_topic` 数值和高度图，不会自动修正坐标漂移。

6. 导航高层也没有自动任务结束状态机。到达目标后的停止、切换到推箱、切换到爬台阶或切回站立，需要由上层 Agent/`llmservice` 根据任务状态管理。

7. 实机首次部署建议按顺序验证：先只启动底层站立，再切 `rl_controller`；确认 `/nav_cmd_vel` 为零或低速；再发 0.3m 小目标；最后再发 1.5m 目标。不要第一次就直接在狭小场景发大目标。

## 常见问题

### 发目标后没有反应

先看 `/push_cmd_vel` 是否有 `push_controller_node` 发布者，并确认 `rl_controller` 订阅的也是 `/push_cmd_vel`。如果没有速度输出，通常是 `push_controller_node` 没启动、ONNX 加载失败、缺少 `onnxruntime`，或缺少有效推箱子观测。

如果 `ros2 topic info /push_box_obs -v` 显示 `unitree_go/msg/HeightMap`，这是正常的原始观测；不要让 `push_controller` 直接订阅这个话题。默认 bringup 会订阅 `/push_box_obs_float`，避免 `/push_box_obs` 出现 `Float32MultiArray` 和 `HeightMap` 同名多类型冲突。

MuJoCo 仿真中，如果刚删除了内部默认 push goal，只发 `/push_box_goal_pose` 后仍没反应，先确认 MuJoCo 已经重启并加载了新 bridge 代码：

```bash
cd /home/xcj/work/Sim2Real/Mujoco
bash run_mujoco.sh
```

然后检查目标文件和观测：

```bash
cat /tmp/mujoco_go2_control/push_box_goal.txt
ros2 topic hz /push_box_obs_float
ros2 topic hz /push_cmd_vel
```

正常情况下，`mujoco_ros2_bridge.py` 启动时会自动清理旧目标文件。如果调试时怀疑旧进程没有退出干净，也可以手动清理：

```bash
rm -f /tmp/mujoco_go2_control/push_box_goal.txt
```

### `/push_box_obs_float` 有一帧后停止

节点不会一直复用旧观测。超过 `push_obs_timeout_sec` 后，旧 `/push_box_obs_float` 会失效。需要让 MuJoCo/感知源持续发布原始 `/push_box_obs`，或启用 fallback 话题。

### 容器内缺少 onnxruntime

当前 Dockerfile 已安装 Python 版 `onnxruntime==1.22.0`。重新 build 镜像后验证：

```bash
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
```
