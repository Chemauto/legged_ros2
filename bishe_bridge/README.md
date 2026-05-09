# bishe_bridge

`bishe_bridge` 是 FinalProject 与 Sim2Sim / Sim2Real 之间的 ROS2 话题适配包。  
它把 MuJoCo / 真机侧状态发布成 FinalProject WebSocket bridge 需要的 `/go2/*` topic，同时把 Planner 发出的控制 topic 写入控制文件，方便底层策略进程读取。

## Topic

发布状态：

- `/go2/odom`：`nav_msgs/Odometry`，机器人位姿，见 [`odom_node.py`](bishe_bridge/odom_node.py)
- `/go2/box_pose`：`geometry_msgs/PoseStamped`，箱子世界坐标，见 [`box_pose_node.py`](bishe_bridge/box_pose_node.py)
- `/go2/scene_objects`：`std_msgs/String`，JSON 场景物体列表，见 [`scene_objects_node.py`](bishe_bridge/scene_objects_node.py)
- `/go2/skill_status`：`std_msgs/String`，JSON 技能执行状态，见 [`skill_status_node.py`](bishe_bridge/skill_status_node.py)

订阅控制：

- `/go2/skill_command`：`std_msgs/String`，技能命令，见 [`skill_command_node.py`](bishe_bridge/skill_command_node.py)
- `/go2/cmd_vel`：`geometry_msgs/Twist`，速度命令，见 [`cmd_vel_node.py`](bishe_bridge/cmd_vel_node.py)
- `/go2/goal_pose`：`geometry_msgs/PoseStamped`，目标点，见 [`goal_pose_node.py`](bishe_bridge/goal_pose_node.py)

## Build

```bash
cd /home/xcj/work/IsaacLab/legged_ws
colcon build --packages-select bishe_bridge
source install/setup.bash
```

## Run

```bash
ros2 launch bishe_bridge bishe_bridge.launch.py
```

带静态场景物体启动：

```bash
ros2 launch bishe_bridge bishe_bridge.launch.py \
  static_scene_objects:='[{"id":"platform_0","type":"platform","center":[2.0,0.0,0.15],"size":[1.0,1.0,0.3],"movable":false}]'
```

## Check ROS2 Topics

如果当前 shell 继承了 Unitree 的 `CYCLONEDDS_URI`，`ros2 topic list` 可能看不到 `/go2/*`。  
检查 Planner 需要的话题前，先清掉它：

```bash
source /opt/ros/jazzy/setup.bash
source /home/xcj/work/IsaacLab/legged_ws/install/setup.bash
unset CYCLONEDDS_URI

ros2 node list
ros2 topic list | grep /go2
```

应能看到：

```text
/go2/odom
/go2/box_pose
/go2/scene_objects
/go2/skill_status
/go2/skill_command
/go2/cmd_vel
/go2/goal_pose
```

启动 `robot_service.py` 时也建议使用这个清掉 `CYCLONEDDS_URI` 的 ROS2 环境，否则 WebSocket bridge 可能看不到 `/go2/*`。

## Parameters

- `domain_id`：DDS domain，默认 `0`
- `interface`：DDS 网卡，默认 `lo`
- `publish_hz`：状态发布频率，默认 `10.0`
- `control_dir`：控制文件输出目录，默认 `/tmp/mujoco_go2_control`
- `static_scene_objects`：静态平台/障碍物 JSON

控制文件示例：

```text
/tmp/mujoco_go2_control/
  skill_command.json
  model_use.txt
  velocity.txt
  goal.txt
  cmd_vel.txt
```

## Command Path To MuJoCo RL

Planner 的 `walk_skill` 会经过：

```text
robot_service.py
  -> /go2/cmd_vel
  -> cmd_vel_node.py 写 velocity.txt
  -> rl_cmd_vel_writer.py 发布 rl_cmd_vel
  -> legged_rl_controller 读取 rl_cmd_vel
```

当前 `go2_description/config/ros2_control/rl.yaml` 中 RL 控制器订阅的是 `rl_cmd_vel`，所以 `rl_cmd_vel_writer.py` 必须和 MuJoCo / go2 控制器在同一个 Unitree/CycloneDDS 环境中运行。默认 launch 已经包含这个进程。
