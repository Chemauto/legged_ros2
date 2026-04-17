# legged_ros2 安装指南

本文档介绍如何安装并配置 `legged_ros2`，用于四足/双足机器人控制、仿真和部署。

[TOC]

## 本地运行

### 环境

- Ubuntu 22.04 LTS
- ROS 2 Jazzy

> 说明：当前中文文档按当前代码分支整理，并以 `jazzy` 环境为主。如果你使用的是 `humble`，将下面命令里的 `jazzy` 按需替换为 `humble` 即可。

### 前置依赖

在安装 `legged_ros2` 之前，请先准备以下依赖：

1. **ROS 2 Jazzy**  
   参考 ROS 2 官方安装文档完成 Jazzy 安装。
2. **ROS 2 Control**  
   安装 ros2_control 相关包：
   ```bash
   sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
   ```
3. **Unitree ROS 2**  
   按照 [Unitree ROS 2 仓库](https://github.com/unitreerobotics/unitree_ros2) 的说明下载并编译。
   - 根据你的实际环境修改 `setup.sh`（真机）或 `setup_local.sh`（仿真）。
   - 重点检查 ROS 发行版、`unitree_ros2/cyclonedds_ws/install/setup.bash` 路径以及网络配置。
   - 后续很多命令都需要先 source `unitree_ros2` 的对应 setup 脚本。

### 安装步骤

1. **克隆仓库**
   ```bash
   cd ~/legged_ws/src
   git clone https://github.com/zitongbai/legged_ros2.git
   ```
2. **下载 ONNX Runtime** 到 `third_party` 目录
   ```bash
   cd ~/legged_ws/src/legged_ros2/third_party
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
   tar -xvzf onnxruntime-linux-x64-1.22.0.tgz
   ```
3. **用 `rosdep` 安装依赖**
   ```bash
   cd ~/legged_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. **编译工作区**
   ```bash
   source path/to/your/unitree_ros2/setup.sh  # 仿真时改为 setup_local.sh
   colcon build --symlink-install
   ```

如果你在当前 `jazzy2` 分支下运行 Go2 RL，并且使用了高程图观测，还需要确保：

- 已经 source ROS Jazzy 环境；
- 已经 source Unitree 的 `cyclonedds_ws/install/setup.bash`；
- `unitree_go` 消息包可以被 CMake 正常找到。

安装完成后，可以继续阅读 [使用指南](usage_zh.md)。

## Docker 运行

如果你希望通过 Docker 安装并运行 `legged_ros2`，可以参考下面的方式。

### 构建运行镜像

```bash
docker build --network host \
  --build-arg HTTP_PROXY=$HTTP_PROXY \
  --build-arg HTTPS_PROXY=$HTTPS_PROXY \
  --build-arg ALL_PROXY=$ALL_PROXY \
  --build-arg http_proxy=$http_proxy \
  --build-arg https_proxy=$https_proxy \
  --build-arg all_proxy=$all_proxy \
  -f src/legged_ros2/docker/Dockerfile \
  -t legged-ros2:jazzy .
```

### 启动容器

```bash
docker run --rm -it --network host legged-ros2:jazzy
```

进入容器后，根据用途选择一个 setup 脚本：

```bash
source /root/legged_ws/setup.sh
# 或
source /root/legged_ws/setup_local.sh
```

## 下一步

安装完成后，继续阅读 [使用指南](usage_zh.md)。
