# llmservice - Agent 到实机 ROS2 的服务端

`llmservice` 是 FQPlanner 和 `legged_ros2` 之间的 WebSocket 服务桥。
它复用 FQPlanner 已有的 WebSocket 协议，把 Agent 规划出的技能命令转换成 ROS2 topic。
本文档只描述实机部署：FQPlanner 跑在本机，`llmservice` 跑在机器人部署端电脑或机器人 Docker 内。

## 数据流

```text
FQPlanner slaver/tools/ws_client.py
  -> ws://<robot_deploy_ip>:8765
  -> llmservice/robot_service.py
  -> ROS2 topics
```

FQPlanner 必须通过 `ROBOT_WS_URL` 指向机器人部署端 IP。

导航链路：

```text
Agent nav(x, y, z)
  -> /go2/goal_pose
  -> nav_controller_node
  -> /nav_cmd_vel
  -> rl_controller
```

状态回传链路：

```text
/odom
/go2/box_pose
/go2/skill_status
/go2/scene_objects
  -> llmservice
  -> WebSocket state/feedback
  -> FQPlanner
```

## 默认 ROS2 topic

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--odom-topic` | `/odom` | 机器人里程计 |
| `--goal-pose-topic` | `/go2/goal_pose` | 导航目标位姿 |
| `--skill-command-topic` | `/go2/skill_command` | 技能模式命令 |
| `--cmd-vel-topic` | `/cmd_vel` | 直接速度技能使用；导航不走这个 topic |
| `--box-pose-topic` | `/go2/box_pose` | 箱子位姿 |
| `--skill-status-topic` | `/go2/skill_status` | 技能状态 |
| `--scene-objects-topic` | `/go2/scene_objects` | 场景物体 |

注意：`bringup_nav.launch.py` 内部会让导航高层和低层使用 `/nav_cmd_vel`，避免遥控器 `/cmd_vel` 零速度覆盖导航输出。
`llmservice` 的 `nav` 技能只发布 `/go2/goal_pose`，不直接发布 `/nav_cmd_vel`。

## 启动顺序

1. 在机器人部署端 Docker 内启动 ROS2 环境。

```bash
cd /root/legged_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

2. 启动机器人导航控制。

```bash
ros2 launch go2_description bringup_nav.launch.py use_rviz:=false use_rqt_cm:=false
```

3. 确认机器人状态 topic 正常。

```bash
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic info /go2/goal_pose -v
ros2 topic info /nav_cmd_vel -v
```

期望 `rl_controller` 能够进入 `active`，`/odom` 有稳定输出；导航模式下 `rl_controller` 应订阅 `/nav_cmd_vel`，避免遥控器 `/cmd_vel` 零速度覆盖导航输出。

4. 在机器人部署端 Docker 内启动本服务端。

```bash
cd /root/legged_ws
python3 llmservice/robot_service.py
```

服务默认监听 `0.0.0.0:8765`，需要保证本机能访问机器人端的 8765 端口。

5. 在本机启动 FQPlanner，并指向机器人部署端。

```bash
cd /home/xcj/work/Sim2Real/FQPlanner
export ROBOT_WS_URL=ws://<robot_deploy_ip>:8765
python3 master/run.py
python3 slaver/run.py
```

实机部署时，FQPlanner 只连接这里的 `llmservice/robot_service.py`。

如果需要管理页面：

```bash
python3 deploy/run.py
```

## 健康检查

服务启动后可以用 FQPlanner 的 WebSocket 客户端，或直接发健康检查：

```json
{"type": "healthcheck"}
```

期望返回：

```json
{
  "type": "health",
  "signal": "SUCCESS",
  "status_json_ready": true
}
```

`status_json_ready` 只有在服务端收到 `/odom` 后才会变成 `true`。如果一直是 `false`，先检查机器人里程计是否在发布 `/odom`，以及 `llmservice` 是否和机器人 ROS2 节点处于同一个 `ROS_DOMAIN_ID`。

ROS2 侧建议检查：

```bash
ros2 topic hz /odom
ros2 topic info /go2/goal_pose -v
ros2 topic info /nav_cmd_vel -v
ros2 topic hz /height_sampler_node/height_map
```

导航模式下期望：

```text
/go2/goal_pose: llmservice 发布，nav_controller_node 订阅
/nav_cmd_vel: nav_controller_node 发布，rl_controller 订阅
/cmd_vel: 遥控器可发布，但 rl_controller 不订阅
```

## WebSocket 命令示例

```json
{"type": "command", "skill": "nav", "args": {"x": 1.0, "y": 0.5, "z": 0.0}}
```

兼容 FQPlanner 里的别名：

```text
navigation -> nav
walk -> walk_skill
push_box -> push
```
