# Installation Guide for Legged ROS2

This document provides step-by-step instructions to install and set up the Legged ROS2 package for controlling, simulating, and deploying legged robots.

## Environment

* Ubuntu 22.04 LTS
* ROS2 Humble 

## Prerequisites

Before installing Legged ROS2, ensure that you have the following prerequisites installed:

1. **ROS2 Humble**: Follow the official ROS2 installation guide for Humble [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. **ROS2 Control**: Install the ROS2 Control packages:
    ```bash
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
    ```
3. **Unitree ROS2**: Download and build the Unitree ROS2 package by following the instructions in the [Unitree ROS2 GitHub repository](https://github.com/unitreerobotics/unitree_ros2).
    * Remember to modify the `setup.sh` (for real robot) or `setup_local.sh` (for simulation) according to your actual situation, including ros distribution, path of `unitree_ros2/cyclonedds_ws/install/setup.bash`, and network configuration. Refer to the official Unitree ROS2 readme for more details.

## Installation

1. **Clone the Repository**: Clone the Legged ROS2 repository into your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/zitongbai/legged_ros2.git 
    ```
2. **Download ONNX Runtime**: Download and unzip the ONNX Runtime to `third_party` folder:
    ```bash
    cd ~/ros2_ws/src/legged_ros2/third_party
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
    tar -xvzf onnxruntime-linux-x64-1.22.0.tgz
    ```
3. **Install Dependencies**: Install the required dependencies using `rosdep`:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
4. **Build the Workspace**: Build your ROS2 workspace using `colcon`:
    ```bash
    source path/to/your/unitree_ros2/setup.sh  # or setup_local.sh for simulation
    colcon build --symlink-install
    ```
