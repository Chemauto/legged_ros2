# 如何修改 legged_ros2 中的 RL 相关部分

本文档聚焦 `legged_ros2` 中 RL 管线的修改方式，尤其是 observation 和 action 接口。

[TOC]

## 1. 当前代码里的 RL 运行时组件

主要文件如下：

- 控制器入口：`legged_rl_controller/src/legged_rl_controller.cpp`
- 环境容器：`isaaclab/envs/manager_based_rl_env.h`
- observation 注册表：`isaaclab/envs/mdp/observations/observations.h`
- observation manager：`isaaclab/manager/observation_manager.h`
- action 注册表：`isaaclab/envs/mdp/actions/joint_actions.h`
- 机器人状态适配层：`legged_rl_controller/legged_articulation.hpp`
- 策略文件：`<robot_description>/config/rl_policy/policy.onnx` 和 `IO_descriptors.yaml`

## 2. 修改前必须保持的约束

下面这些约束必须成立：

1. ONNX 的输入名必须和 `ObservationManager` 输出的 observation group 名一致；
2. `IO_descriptors.yaml` 中 observation 的 `name` 必须在 `observations.h` 中注册；
3. `IO_descriptors.yaml` 中 action 的 `name` 必须在 `joint_actions.h` 中注册；
4. action 维度必须和机器人关节控制维度一致；
5. `config/ros2_control/*.yaml` 中 `rl_controller.joint_names` 的顺序必须和 `IO_descriptors.yaml` 中导出的顺序一致。

## 3. 常见改动场景

### 3.1 只更新策略权重

如果 observation / action 结构没变：

1. 替换 `policy.onnx`；
2. 用同一次导出的 `IO_descriptors.yaml` 一起替换；
3. 不需要修改 C++ 代码。

### 3.2 新增或修改 observation

这是最常见的代码改动路径。

#### 第一步：在 Isaac Lab 中重新导出

- 修改 Isaac Lab 侧 observation 配置；
- 重新导出；
- 直接使用导出的 `IO_descriptors.yaml`。

#### 第二步：在 C++ 中实现并注册 observation

在 `observations.h` 中新增一项：

- 使用 `REGISTER_OBSERVATION(<name>)`
- `<name>` 必须和 `IO_descriptors.yaml` 中 `observations.policy[*].name` 完全一致
- 返回值必须是固定维度的 `std::vector<float>`

#### 第三步：确保运行时有对应数据源

如果新 observation 依赖当前代码还没暴露的数据，则需要同步修改：

- `legged_articulation.hpp` 的 `update()` 路径
- 或 `isaaclab/assets/articulation/articulation.h` 里的 `ArticulationData`

### 3.3 修改 action

当前动作项在 `joint_actions.h` 里实现，默认使用 `joint_position_action`。

如果动作语义变化：

1. 先从 Isaac Lab 导出新的 action descriptors；
2. 在 `joint_actions.h` 中新增或修改 action term；
3. 注册名必须和 descriptor 中的 action `name` 一致；
4. 处理后的动作维度必须等于控制关节数量。

## 4. 当前代码里的重要实现细节

在 `observation_manager.h` 中：

- 每个 observation term 都必须包含 `overloads`
- `flatten_history_dim` 必须为 `true`
- `history_length <= 0` 会在运行时按 `1` 处理
- 如果 `observations` 只有一个 group，则会映射成输入键 `"obs"`

在 `legged_rl_controller.cpp` 中：

- `onnx_model_path` 和 `io_descriptors_path` 不能为空
- `articulations.robot` 必须存在
- joint name 数量会做校验
- 当前运行时校验的是 joint name **数量**，不是精确顺序，因此顺序一致性必须靠配置保证
- 至少需要一个 IMU 接口
- action 维度不匹配会直接报错

## 5. 当前分支新增的高程图链路

当前代码中，Go2 RL 已经新增地形观测接线：

- `legged_rl_controller.cpp` 订阅 `/heightmap`
- 消息类型为 `unitree_go::msg::HeightMap`
- 数据写入 `ArticulationData::height_scan`
- `observations.h` 中新增 `height_scan`
- `IO_descriptors.yaml` 中对应 `height_scan.params.expected_dim: 187`

这意味着如果你的策略包含 `height_scan`：

1. `IO_descriptors.yaml` 必须包含 `height_scan`
2. 运行时必须能拿到 `/heightmap`，或者接受全零回退
3. `expected_dim` 必须和真实输入一致

## 6. IO_descriptors 的使用策略

`IO_descriptors.yaml` 应当由 Isaac Lab 导出，而不是手工长期维护。

推荐流程：

1. 在 Isaac Lab 中训练或更新策略
2. 一起导出 `policy.onnx` 和 `IO_descriptors.yaml`
3. 将两者拷贝到对应机器人 description 包里
4. 只有路径变化时，才修改 launch 参数

除非你在调试内部细节，否则尽量不要手工改 descriptor。

## 7. RL 改动后的检查清单

1. 编译：

```bash
colcon build --packages-select legged_rl_controller go2_description
```

2. 启动 RL 栈；
3. 确认 controller configure 成功，没有 YAML 或 schema 错误；
4. 检查是否出现以下运行时错误：
   - observation term 未注册
   - ONNX 输入名和 observation group 对不上
   - action 维度不匹配
   - `height_scan` 维度不匹配
5. 先用较小动作幅度验证行为是否正常。

## 8. 常见调试信号

- `Observation term '<name>' is not registered.`
  - 说明你需要在 `observations.h` 中增加对应的 `REGISTER_OBSERVATION(<name>)`

- `Input name <...> not found in observations.`
  - 说明 ONNX 输入名和 observation group 名不一致

- `Action size mismatch: action=..., joints=...`
  - 说明动作头维度或关节列表和当前机器人配置不一致

- `height_scan` 相关维度错误或 ONNX 输入维度错误
  - 说明 `HeightMap` 输入长度与 `IO_descriptors.yaml` 中的 `expected_dim` 不一致
