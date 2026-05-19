# legged_ros2 使用说明

本文档说明如何使用 Legged ROS2 控制和仿真腿式机器人。下面以 Unitree Go2 为例，其他腿式机器人可以按同样思路适配。

[TOC]


## Sim2Sim：使用 Unitree Mujoco 仿真


使用 [Unitree Mujoco](https://github.com/unitreerobotics/unitree_mujoco) 做 Sim2Sim 时，按下面步骤操作：

1. **安装 Unitree Mujoco**：先确认已经安装 Unitree Mujoco。安装方法参考 Unitree Mujoco 的 GitHub 仓库。建议使用 C++ 仿真器，性能更好。
2. **修改仿真配置**：根据实际情况修改 `unitree_mujoco` 中的 `simulate/config.yaml`，并把 `use_joystick` 改成 `1`。
    > Unitree Mujoco 默认使用 `ROS_DOMAIN_ID=1`。运行 ROS 2 节点的终端也需要使用同一个 domain：
    > ```bash
    > export ROS_DOMAIN_ID=1
    > ```
3. **启动仿真器**：在 `unitree_mujoco/simulate/build` 目录运行：
    ```bash
    ./unitree_mujoco
    ```
    > 如果启动时报错，可以尝试在一个没有 source 任何 ROS 2 setup 文件的新终端中启动仿真器。
4. **启动控制节点**：打开一个新终端，source `legged_ros2` 工作空间，然后启动控制节点：
    ```bash
    # 本机工作空间
    source path/to/unitree_ros2/setup_local.sh
    source ~/legged_ws/install/setup.bash

    # Docker 工作空间。先清理旧的 CycloneDDS 配置，避免 XML 解析错误。
    # source /root/legged_ws/setup_local.sh

    colcon build --symlink-install
    unset CYCLONEDDS_URI

    export ROS_DOMAIN_ID=1
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

    # 调试时建议先不开 RViz/rqt，减少 CycloneDDS participant 数量。
    ros2 launch go2_description bringup_rl.launch.py use_rviz:=false use_rqt_cm:=false
    ```

    `CYCLONEDDS_URI` 必须保持为单行 XML，不要手动换行。可以用下面命令检查：

    ```bash
    printf '%s\n' "$CYCLONEDDS_URI"
    ```

    如果需要启用 rqt controller manager，先在 Docker 中安装：

    ```bash
    apt update
    apt install -y ros-humble-rqt-controller-manager
    ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
    ```

    `use_rqt_cm:=true` 需要安装 `rqt_controller_manager` 包。如果当前环境没有这个包，先保持 `use_rqt_cm:=false`。

    启动稳定后，另开一个 Docker 终端检查控制器状态：

    ```bash
    ros2 control list_controllers
    ```

    正常情况下应看到 `imu_state_broadcaster` 和 `joint_state_broadcaster` 为 `active`，三个控制器已加载但为 `inactive`：

    ```text
    stand_static_controller inactive
    rl_controller           inactive
    sit_static_controller   inactive
    imu_state_broadcaster   active
    joint_state_broadcaster active
    ```

    然后先激活站立控制器：

    ```bash
    ros2 control switch_controllers --activate stand_static_controller --strict
    ```

    等机器人站稳后切换到 RL 控制器：

    ```bash
    ros2 control switch_controllers --deactivate stand_static_controller --activate rl_controller --strict
    ```

    发布速度命令测试 RL 控制：

    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
    ```

    检查是否有底层控制命令输出：

    ```bash
    ros2 topic echo /lowcmd --once
    ```

仿真启动后，可以用手柄控制机器人：
```text
LB + A: 站起
LB + B: 坐下
LB + X: 启动 RL 控制器
LB + RB: 停止所有控制器
```

## Sim2Real：连接真实机器人

下面的操作是在外部 PC 上执行，不是在机器人机载电脑上执行。

> **警告**：真实机器人调试有风险。开始前请确认：
> - 周围环境开阔，没有障碍物和人员靠近。
> - 机器人放在平整稳定的地面上。
> - 已经熟悉急停流程。
> - 旁边有人能在机器人失稳时及时扶住或处理。

> **免责声明**：本软件作者和贡献者不对使用过程中的损坏、受伤或损失负责。请自行承担风险，并始终优先保证安全。


### 硬件连接

用网线连接 PC 和机器人机载电脑。确保 PC 的有线网卡 IP 地址和子网掩码设置正确，能和机器人通信。具体网络配置请参考 Unitree 官方文档。

### 启动真实机器人控制

1. **关闭 Go2 自带的主运动控制服务**：可以使用 `unitree_sdk2` 里的 `go2_stand_example` 停止机器人自带运动控制服务：
    ```bash
    # 运行前确认机器人处于趴下状态
    cd path/to/unitree_sdk2/build/bin
    ./go2_stand_example 
    ```
2. **启动控制节点**：关闭上一步的终端，打开新终端并 source `legged_ros2` 工作空间，然后运行控制节点：
    ```bash
    # 本机工作空间
    source path/to/unitree_ros2/setup.sh
    source ~/legged_ws/install/setup.bash

    # Docker 工作空间：选择连接机器人的有线网卡
    # export NET_IF=<your-robot-network-interface>
    # source /root/legged_ws/setup.sh

    colcon build --packages-select legged_ros2_control legged_ros2_controller legged_rl_controller go2_description legged_mapping --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    # 或者
    colcon build --symlink-install

    ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=false
    ```
    只有在已经安装 `rqt_controller_manager` 后，才把 `use_rqt_cm` 改成 `true`。

真实机器人启动后，可以用手柄控制：
```text
L1 + A: 站起
L1 + B: 坐下
L1 + X: 启动 RL 控制器
L1 + RB: 停止所有控制器
```


## Docker 中的 CycloneDDS 网卡配置

Sim2Sim 和 Sim2Real 使用不同的 setup 脚本：

- Sim2Sim：使用 `/root/legged_ws/setup_local.sh`。
- Sim2Real：使用 `/root/legged_ws/setup.sh`，并选择连接机器人的网卡。

在 Docker 中使用 `/root/legged_ws/setup.sh` 时，需要为 CycloneDDS 选择正确的网络接口。脚本的选择逻辑是：

- 如果设置了 `NET_IF`，就使用 `NET_IF` 指定的网卡。
- 如果没有设置 `NET_IF`，就自动读取默认路由网卡；如果读取失败，才回退到 `eth0`。

示例：

```bash
# 自动选择默认路由网卡
source /root/legged_ws/setup.sh
```

```bash
# 手动指定连接机器人的网卡
export NET_IF=<your-robot-network-interface>
source /root/legged_ws/setup.sh
```

如果 ROS 2 打印下面的错误，说明 CycloneDDS 选择的网卡在容器里不存在：

```text
eth0: does not match an available interface
```

可以用下面命令查看容器里可用的网卡和默认路由：

```bash
ip -o link show
ip route
```

对于 Sim2Sim，建议打开一个新终端后 source `/root/legged_ws/setup_local.sh`。对于 Sim2Real，先把 `NET_IF` 设置成连接机器人的有线网卡，再 source `/root/legged_ws/setup.sh`。

