# 如何把你的机器人接入 legged_ros2

## 机器人描述包

在终端中创建一个 ROS 2 包：

```bash
ros2 pkg create --build-type ament_cmake <your_robot_description> --dependencies legged_ros2_controller legged_ros2_control
```

在包内创建目录：

```bash
cd <your_robot_description>
mkdir urdf
mkdir meshes
mkdir config
mkdir rviz2
mkdir launch
```

然后在 `CMakeLists.txt` 中加入：

```cmake
install(
  DIRECTORY config launch meshes urdf rviz2
  DESTINATION share/${PROJECT_NAME}
)
```

并编译工作区：

```bash
colcon build --packages-select <your_robot_description> --symlink-install
```

### URDF/XACRO

准备机器人 URDF，可以来自厂商，也可以来自你自己建的 CAD 模型。

然后基于 URDF 创建 xacro。可以参考：

- `g1_description/urdf/g1_29dof_lock_waist_rev_1_0.urdf`
- `g1_description/urdf/g1_29dof_lock_waist_rev_1_0_macro.urdf.xacro`

如果有 mesh 文件，记得检查路径，例如：

```text
package://g1_description/meshes/pelvis_contour_link.STL
```

你还需要一个包含 `ros2_control` 信息的 xacro，用来定义：

- hardware plugin
- command interface
- state interface

可以参考 `g1_description/urdf/g1_29dof_lock_waist_rev_1_0.ros2_control.urdf.xacro`。

生成后可以这样验证：

```bash
xacro your_robot.urdf.xacro > test.urdf
```

如果没有报错，说明 xacro 基本可用。

### 用 RViz2 可视化

你可以通过 RViz2 检查模型和关节是否合理。

1. 在 `rviz2/` 下创建 RViz 配置文件
2. 在 `launch/` 下创建启动文件，例如参考 `g1_description/launch/view_robot.launch.py`
3. 编译并运行：

```bash
colcon build --packages-select <your_robot_description> --symlink-install
source install/setup.bash
ros2 launch <your_robot_description> view_robot.launch.py
```

如果机器人能在 RViz2 正常显示，说明 URDF/Xacro 基本正确。

## 硬件接口

这一部分说明如何在 `legged_ros2_control` 中实现你的 ROS 2 control 硬件接口。建议先阅读 Go2 的实现，再按相同结构扩展。

### 目录结构

在 `legged_ros2_control` 下创建机器人目录，例如：

```bash
src/legged_ros2/legged_ros2_control/include/legged_ros2_control/robots/<your_robot>/
src/legged_ros2/legged_ros2_control/src/robots/<your_robot>/
```

通常包括：

- `<robot>_lowlevel_node.hpp/.cpp`
- `<robot>_system_interface.hpp/.cpp`
- `<robot>_wireless_controller.hpp/.cpp`
- `motor_crc*.h/.cpp`
- `<robot>_main_loop.cpp`

### 实现重点

1. **LowLevelNode**
   - 订阅 lowstate
   - 发布 `/lowcmd`
   - 在发送前计算 CRC
2. **SystemInterface**
   - 在 `on_init()` 中创建底层节点
   - 在 `build_joint_data_()` 中建立 joint name 到底层电机编号的映射
   - `read()` 读关节和 IMU
   - `write()` 写回 lowcmd
3. **Wireless Controller**（可选）
   - 读取手柄输入
   - 发布 `/cmd_vel`
   - 支持控制器切换

### 构建和插件注册

实现后还需要更新：

- `legged_ros2_control/CMakeLists.txt`
- `legged_ros2_control/src/robots/<your_robot>/CMakeLists.txt`
- `legged_ros2_control/legged_ros2_control_plugins.xml`
- `legged_ros2_control/package.xml`

### 与 URDF 对齐

- joint name 必须和 `ros2_control` 配置完全一致
- IMU 数量必须和 `ros2_control` 里的 sensor 配置一致

最后可以这样构建和运行：

```bash
colcon build --packages-select legged_ros2_control
ros2 run legged_ros2_control <your_robot>_main_loop
```
