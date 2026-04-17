# legged_ros2 使用指南

本文档说明如何使用 `legged_ros2` 控制和仿真足式机器人。这里以 Unitree Go2 为例，其他机器人可以按相同思路适配。

[TOC]

## 使用 Unitree Mujoco 进行仿真

如果你希望用 [Unitree Mujoco](https://github.com/unitreerobotics/unitree_mujoco) 做仿真，可以按下面步骤操作：

1. **安装仿真器**  
   按照 Unitree Mujoco 仓库说明完成安装。建议优先使用 C++ 版本仿真器以获得更好的性能。
2. **修改配置**  
   按实际情况修改 `unitree_mujoco/simulate/config.yaml`。通常需要把 `use_joystick` 改为 `1`。  
   > `unitree_mujoco` 默认使用 `ROS_DOMAIN_ID=1`，请确保运行 ROS 2 节点的终端使用相同的 `ROS_DOMAIN_ID`：
   > ```bash
   > export ROS_DOMAIN_ID=1
   > ```
3. **启动仿真**
   在 `unitree_mujoco/simulate/build` 下运行：
   ```bash
   ./unitree_mujoco
   ```
   > 如果报错，可以尝试在一个没有 source ROS 2 环境的新终端里启动仿真器。
4. **启动控制**
   打开新终端，source `legged_ros2` 工作区并启动控制节点：
   ```bash
   cd ~/legged_ws
   source path/to/unitree_ros2/setup_local.sh
   colcon build --packages-select legged_ros2_control legged_ros2_controller legged_rl_controller go2_description --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
   source ~/legged_ws/install/setup.bash
   export ROS_DOMAIN_ID=1
   ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
   ```

   > **重要**：
   > - 请在工作区根目录 `~/legged_ws` 下执行 `colcon build`，不要在 `~/legged_ws/src/legged_ros2` 里直接编译。
   > - 启动前必须 `source ~/legged_ws/install/setup.bash`，否则 `ros2` 只能看到 `/opt/ros/jazzy` 和 `unitree_ros2` 的包，看不到 `go2_description`。
   > - 如果你在 conda 环境里编译，建议显式加上 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`，避免 `catkin_pkg` 等 Python 环境问题。

然后你可以通过手柄控制机器人：

```text
LB + A: 起立
LB + B: 坐下
LB + X: 启动 RL 控制器
LB + RB: 停止所有控制器
```

### 当前代码补充

当前分支的 Go2 RL 链路已经补上了高程图输入：

- `rl_controller` 会订阅 `/heightmap`
- 观测中新增 `height_scan`
- `IO_descriptors.yaml` 中已包含对应的 `height_scan` 项

因此在仿真或真机运行时，如果你的策略依赖地形观测，请确认 `/heightmap` 正常发布；如果没有该消息，当前代码会回退为全零观测，而不会直接崩溃。

### 启动后控制器状态说明

`ros2 launch go2_description bringup_rl.launch.py` 是前台常驻进程，不会自动退出到命令行，这属于正常现象，不代表程序卡死。

当前 `bringup_rl.launch.py` 会先把以下控制器加载为 `inactive`：

- `stand_static_controller`
- `sit_static_controller`
- `rl_controller`

而 `joint_state_broadcaster` 和 `imu_state_broadcaster` 会自动进入 `active`。因此启动后如果看到运动控制器仍然是 `inactive`，这也是当前配置的正常行为。

你可以在新终端检查状态：

```bash
source /opt/ros/jazzy/setup.bash
source path/to/unitree_ros2/setup_local.sh
source ~/legged_ws/install/setup.bash
ros2 control list_hardware_components
ros2 control list_controllers
```

如果想手动激活站立控制器：

```bash
ros2 control switch_controllers -c /controller_manager \
  --activate stand_static_controller \
  --strict
```

如果当前 `stand_static_controller` 已经激活，想切到 RL 控制器：

```bash
ros2 control switch_controllers -c /controller_manager \
  --activate rl_controller \
  --deactivate stand_static_controller \
  --strict
```

注意：

- 不要在 `--strict` 模式下去停用一个本来就是 `inactive` 的控制器，否则整次切换会失败。
- 如果当前没有任何运动控制器是 `active`，通常只写 `--activate` 即可。

## 真机运行

下面所有操作默认都在 **PC 端** 完成，而不是机器人板载电脑。

> **警告**：真机运行有风险。开始前请确保：
> - 周围有足够的安全空间；
> - 地面平整稳定；
> - 你了解急停流程；
> - 旁边有人可以在机器人失稳时及时保护。

> **免责声明**：作者和贡献者不对使用本软件导致的损坏、伤害或损失负责。请自行承担风险，并始终优先考虑安全。

### 硬件连接

通过网线将 PC 和机器人板载电脑连接，正确配置 PC 网卡 IP 和子网掩码，确保能和机器人通信。具体可参考 [Unitree 文档](https://support.unitree.com/home/en/developer/Quick_start)。

### 启动机器人控制

1. **关闭 Go2 自带运动控制服务**  
   可以用 `unitree_sdk2` 里的 `go2_stand_example` 停掉机器人原始运动控制：
   ```bash
   cd path/to/unitree_sdk2/build/bin
   ./go2_stand_example
   ```
   > 运行前请确认机器人已经处于安全姿态，例如趴下。
2. **启动控制栈**
   在新的终端中执行：
   ```bash
   cd ~/legged_ws
   source path/to/unitree_ros2/setup.sh
   colcon build --packages-select legged_ros2_control legged_ros2_controller legged_rl_controller go2_description --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
   source ~/legged_ws/install/setup.bash
   ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
   ```

   > **重要**：
   > - 请从 `~/legged_ws` 根目录编译并 source 工作区。
   > - 如果没有 `source ~/legged_ws/install/setup.bash`，启动时常见报错就是：
   >   `Package 'go2_description' not found`
   > - 如果你之前在 `src/legged_ros2` 目录里执行过 `colcon build`，建议回到 `~/legged_ws` 重新编译一次，避免 `install` 目录不一致。

然后可以通过手柄控制：

```text
L1 + A: 起立
L1 + B: 坐下
L1 + X: 启动 RL 控制器
L1 + RB: 停止所有控制器
```

## Docker 中 CycloneDDS 的网卡设置

当你在 Docker 中使用 `/root/legged_ws/setup.sh` 时，需要为 CycloneDDS 指定正确的网络接口。

脚本的处理逻辑如下：

- 如果设置了 `NET_IF`，就直接使用该接口；
- 否则自动检测默认路由对应的接口，失败时回退到 `eth0`。

示例：

```bash
# 使用默认检测到的网卡
source /root/legged_ws/setup.sh
```

```bash
# 手动指定网卡
export NET_IF=eth0
source /root/legged_ws/setup.sh
```
