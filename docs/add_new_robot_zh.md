# 如何在 legged_ros2 中新增机器人

本文档说明如何将新的足式机器人接入当前的 `legged_ros2` 栈。

> 注意：`config/rl_policy/IO_descriptors.yaml` 由 Isaac Lab 自动导出，**不要手工长期维护**。除非你在做调试，并且完全理解其影响。

[TOC]

## 1. 你通常需要改哪些地方

新增一个机器人，通常需要更新下面几个位置：

1. `legged_robot_description`  
   URDF/Xacro、launch、controller YAML
2. `legged_ros2_control`  
   硬件 / SystemInterface 插件
3. `legged_rl_controller` / `legged_ros2_controller`  
   控制器配置接线
4. `legged_mapping`（可选）  
   如果传感器或 TF 拓扑不同

## 2. 创建机器人描述包

通常可以参考 `go2_description` 的结构创建新的 description 包：

- `urdf/`：机器人模型、`ros2_control` 标签、transmission
- `config/ros2_control/*.yaml`：controller manager 与控制器配置
- `config/main_loop/*.yaml`：手柄绑定与控制器切换行为
- `launch/bringup_*.launch.py`：RL、static、broadcaster 等启动方式
- `config/rl_policy/`：策略文件和描述文件

### 一致性检查

- URDF 里的关节名必须和 ros2_control YAML 完全一致。
- ros2_control YAML 中的 IMU 名称（例如 `imu`）必须能在 URDF 的 `ros2_control` 配置里找到。
- YAML 里的控制器列表必须和实际加载的插件匹配。
- 对于 `rl_controller`，`config/ros2_control/*.yaml` 中的 `joint_names` 顺序必须和 Isaac Lab 导出的策略顺序保持一致，重点看 `IO_descriptors.yaml` 中的 `articulations.robot.joint_names` 和 action 对应的 joint 顺序。

## 3. 在 `legged_ros2_control` 中实现硬件接口

在下面路径新增机器人后端：

- `src/legged_ros2/legged_ros2_control/src/robots/<your_robot>/`

典型文件包括：

- `<your_robot>_system_interface.cpp`
- `<your_robot>_lowlevel_node.cpp`（如果需要）
- `<your_robot>_main_loop.cpp`（如果主流程和 Go2 不同）
- `CMakeLists.txt`

### 必做接入点

1. 在 `legged_ros2_control/CMakeLists.txt` 中增加对应子目录。
2. 在 `legged_ros2_control/legged_ros2_control_plugins.xml` 中导出插件类。
3. 确保你的类继承自 `legged::LeggedSystemInterface`，并正确导出关节和 IMU 接口。

## 4. 接通启动流程

创建或修改启动文件（例如 `bringup_rl.launch.py`），完成以下接线：

- 机器人描述（由 xacro 生成的 `robot_description`）
- ros2_control YAML
- main-loop YAML
- RL 模型路径（`policy.onnx`）
- IO 描述路径（`IO_descriptors.yaml`）

对于新机器人，通常需要在新的 description 包里提供一套独立 launch。

## 5. 新机器人的 RL 策略文件

在 RL 模式下，通常需要将策略文件放进机器人 description 包，例如：

- `config/rl_policy/policy.onnx`
- `config/rl_policy/IO_descriptors.yaml`

注意：

- `IO_descriptors.yaml` 应当和 ONNX 一起从 Isaac Lab 同一次导出中获得；
- `policy.onnx` 和 `IO_descriptors.yaml` 必须来自同一次导出；
- 尽量不要手工改 descriptors。

如果你的策略包含地形观测，例如当前 Go2 分支里的 `height_scan`，还要额外保证：

- 运行时确实有对应的观测源；
- C++ 端已经注册该 observation term；
- `expected_dim` 和实际消息维度一致。

## 6. 控制器级别检查清单

在启动 RL 之前，请确认：

1. `rl_controller.joint_names` 顺序和 `IO_descriptors.yaml` 完全一致；
2. `imu_names` 有效且能被控制器拿到；
3. 关节刚度、阻尼、默认姿态在硬件侧和 descriptors 中保持一致；
4. `cmd_vel` topic 和对应范围参数已经配置好；
5. 如果有 `height_scan` 之类额外观测，输入链路已经就绪。

## 7. 最小验证步骤

1. 编译：

```bash
colcon build --packages-select legged_ros2_control legged_ros2_controller legged_rl_controller <your_description_pkg>
```

2. 先启动 broadcaster-only 模式。
3. 检查接口：

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

4. 先激活 static controller，确认安全姿态正常，再切换到 RL controller。
5. 检查是否出现：

- 关节顺序不匹配
- 动作维度不匹配
- IMU 缺失
- 观测项未注册

## 8. 常见失败模式

- 硬件和 RL descriptors 的关节名 / 顺序不一致；
- IMU 接口不存在，导致 controller configure 失败；
- `legged_ros2_control_plugins.xml` 中插件类名写错；
- ONNX 和 descriptors 不是同一次导出；
- 地形观测类策略启用了 `height_scan`，但运行时没有正确提供 `/heightmap`。
