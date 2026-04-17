# legged_mapping 建图说明

`legged_mapping` 包提供的是足式机器人地图使用相关的辅助工具，本身**不直接执行建图算法**，而是为建图流程提供配套节点和 TF 工具。

这里给出的是：在带有激光雷达的足式机器人上，结合 FAST-LIO 2 使用建图的参考流程。

大多数操作都建议在 **PC 端** 完成，而不是机器人板载电脑，以获得更好的性能，并避免和机器人本身运行的其他进程冲突。

当前支持情况：

- [x] Unitree Go2 + MID360
- [ ] Unitree G1 + MID360
- [ ] Unitree Go2 + XT16

## 使用 FAST-LIO 2 进行建图

### Unitree Go2 + MID360

本节介绍如何在 Unitree Go2 + MID360 上配置 FAST-LIO 2 建图。

#### 硬件连接

将 MID360 安装在 Go2 机器人上，并通过指定线缆连接到机器人板载电脑；再通过网线把你的 PC 连到机器人板载电脑。

```mermaid
graph LR;
    A[PC] <-- Ethernet --> B[Go2];
    B <-- Cable --> C[MID360];
```

由于 MID360 通过网络通信，因此你可以直接在 PC 上接收 MID360 数据，并通过 Livox Viewer 2 修改传感器参数。

#### 前置依赖

1. 如果你使用的是 Unitree 配套的 MID360，一般可以跳过初始网络配置；否则请先通过 Livox Viewer 2 把 MID360 的网络配置改成与你的环境一致。一个常见配置如下：
   ```text
   Lidar IP: 192.168.123.20
   Gateway Address: 192.168.123.1
   Lidar Info IP: 192.168.123.70
   ```
2. 在 PC 上安装 `Livox-SDK2`，参考 [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)。
3. 在 PC 上安装 `livox_ros_driver2`，参考 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)。建议放在独立工作区，例如 `~/livox_ws`，并使用 ROS 2 Jazzy/Humble 与本工程保持一致。
4. 修改 `livox_ros_driver2/config/MID360_config.json` 的网络配置，保证与你的实际 PC/MID360 网段一致。下面是一个示例：
   ```json
   "MID360": {
       "lidar_net_info" : {
       "cmd_data_port": 56100,
       "push_msg_port": 56200,
       "point_data_port": 56300,
       "imu_data_port": 56400,
       "log_data_port": 56500
       },
       "host_net_info" : {
       "cmd_data_ip" : "192.168.123.200",
       "cmd_data_port": 56101,
       "push_msg_ip": "192.168.123.200",
       "push_msg_port": 56201,
       "point_data_ip": "192.168.123.200",
       "point_data_port": 56301,
       "imu_data_ip" : "192.168.123.200",
       "imu_data_port": 56401,
       "log_data_ip" : "",
       "log_data_port": 56501
       }
   },
   "lidar_configs" : [
       {
       "ip" : "192.168.123.20",
       "pcl_data_type" : 1,
       "pattern_mode" : 0,
       "extrinsic_parameter" : {
           "roll": 0.0,
           "pitch": 0.0,
           "yaw": 0.0,
           "x": 0,
           "y": 0,
           "z": 0
       }
       }
   ]
   ```
5. 在 PC 上安装 `FAST-LIO 2`，参考 [FAST-LIO 2 ROS2 分支](https://github.com/hku-mars/FAST_LIO/tree/ROS2)。建议放在独立工作区，例如 `~/fast_lio_ws`。  
   > `livox_ros_driver2` 和 `FAST-LIO 2` 建议使用独立工作区，避免相互污染。  
   > 在编译 `FAST-LIO 2` 之前，先 source `livox_ws/install/setup.bash`。

#### 启动建图

通常需要 4 个终端：

1. **终端 1**：启动 `livox_ros_driver2`，接收 MID360 数据
   ```bash
   source path/to/unitree_ros2/setup.sh
   source ~/livox_ws/install/setup.bash
   ros2 launch livox_ros_driver2 msg_MID360_launch.py
   ```
2. **终端 2**：启动 `FAST-LIO 2`
   ```bash
   source path/to/unitree_ros2/setup.sh
   source ~/fast_lio_ws/install/setup.bash
   ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
   ```
3. **终端 3**：启动 `dual_imu_static_tf_node`，发布静态 TF
   ```bash
   source path/to/unitree_ros2/setup.sh
   source ~/legged_ws/install/setup.bash
   ros2 run legged_mapping dual_imu_static_tf_node
   ```
4. **终端 4**（可选）：广播机器人 TF 并在 RViz2 中可视化
   ```bash
   source path/to/unitree_ros2/setup.sh
   source ~/legged_ws/install/setup.bash
   ros2 launch go2_description bringup_broadcasters.launch.py
   ```

### Unitree G1 + MID360

后续补充。

### Unitree Go2 + XT16

后续补充。
