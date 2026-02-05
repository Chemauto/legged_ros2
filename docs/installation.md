# Installation Guide for Legged ROS2

This document provides step-by-step instructions to install and set up the Legged ROS2 package for controlling, simulating, and deploying legged robots.

[TOC]


## Local Run

### Environment

- Ubuntu 22.04 LTS
- ROS 2 Humble

### Prerequisites

Before installing Legged ROS2, make sure the following prerequisites are ready:

1. **ROS 2 Humble**: Follow the official ROS 2 installation guide for Humble [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. **ROS 2 Control**: Install ROS 2 Control packages:
   ```bash
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   ```
3. **Unitree ROS 2**: Download and build Unitree ROS 2 by following the instructions in the [Unitree ROS 2 repository](https://github.com/unitreerobotics/unitree_ros2).
   - Remember to modify `setup.sh` (for real robot) or `setup_local.sh` (for simulation) for your setup, including ROS distribution, path to `unitree_ros2/cyclonedds_ws/install/setup.bash`, and network configuration.
   - Many operations later will require sourcing either `setup.sh` or `setup_local.sh` in `unitree_ros2`.

### Installation

1. **Clone the repository**:
   ```bash
   cd ~/legged_ws/src
   git clone https://github.com/zitongbai/legged_ros2.git
   ```
2. **Download ONNX Runtime** to the `third_party` directory:
   ```bash
   cd ~/legged_ws/src/legged_ros2/third_party
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
   tar -xvzf onnxruntime-linux-x64-1.22.0.tgz
   ```
3. **Install dependencies** with `rosdep`:
   ```bash
   cd ~/legged_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. **Build the workspace** with `colcon`:
   ```bash
   source path/to/your/unitree_ros2/setup.sh  # or setup_local.sh for simulation
   colcon build --symlink-install
   ```

After installation, continue with the [Usage Guide](usage.md).

## Docker Run

If you prefer to use Docker for installation and running Legged ROS2, follow the instructions below.

### Build a production image

Build the production image with the Dockerfile in the repository:

```bash
docker build --network host \
  --build-arg HTTP_PROXY=$HTTP_PROXY \
  --build-arg HTTPS_PROXY=$HTTPS_PROXY \
  --build-arg ALL_PROXY=$ALL_PROXY \
  --build-arg http_proxy=$http_proxy \
  --build-arg https_proxy=$https_proxy \
  --build-arg all_proxy=$all_proxy \
  -f src/legged_ros2/docker/Dockerfile \
  -t legged-ros2:humble .
```

### Start a container

```bash
docker run --rm -it --network host legged-ros2:humble
```

Inside the container, choose one setup script based on your use case:

```bash
source /root/legged_ws/setup.sh
# or
source /root/legged_ws/setup_local.sh
```

<!-- ### Verify installation with the dedicated verification Dockerfile

If you only want to verify the installation pipeline, use `Dockerfile.installation.verify`:

```bash
docker build --network host \
  --build-arg HTTP_PROXY=$HTTP_PROXY \
  --build-arg HTTPS_PROXY=$HTTPS_PROXY \
  --build-arg ALL_PROXY=$ALL_PROXY \
  --build-arg http_proxy=$http_proxy \
  --build-arg https_proxy=$https_proxy \
  --build-arg all_proxy=$all_proxy \
  -f src/legged_ros2/docker/Dockerfile.installation.verify \
  -t legged-install-verify:humble .
``` -->

## What's Next?

After installation, continue with the [Usage Guide](usage.md).

