# Usage for legged_ros2

This document provides instructions on how to use the Legged ROS2 package for controlling and simulating legged robots. Here we take Unitree Go2 as an example, but the same principles can be applied to other legged robots with appropriate modifications.

[TOC]


## Simulation with Unitree Mujoco


To simulate a legged robot using [Unitree Mujoco](https://github.com/unitreerobotics/unitree_mujoco), follow these steps:

1. **Installation** Make sure you have Unitree Mujoco installed. Follow the instructions in the [Unitree Mujoco GitHub repository](https://github.com/unitreerobotics/unitree_mujoco). We recommend using the C++ simulator for better performance.
2. **Modify the config** Modify the `simulate/config.yaml` in `unitree_mujoco` according to your actual situation. Change `use_joystick` to `1`. 
    > Note that the ROS_DOMAIN_ID for unitree_mujoco is `1` by default. Make sure to set the same ROS_DOMAIN_ID in your terminal before running any ROS2 nodes:
    > ```bash
    > export ROS_DOMAIN_ID=1
    > ```
3. **Bringup the simulation** In `unitree_mujoco/simulate/build`, run the following command to start the simulation:
    ```bash
    ./unitree_mujoco
    ```
    > Sometimes error may occur, you can try to run the simulator in a new terminal without sourcing any ROS2 related setup files.
4. **Control the robot** Open a new terminal, source the `legged_ros2` workspace and run the control node:
    ```bash
    source path/to/unitree_ros2/setup_local.sh # Source ROS2 and change DDS implementation to avoid DDS conflicts
    source ~/legged_ws/install/setup.bash # Source legged_ros2 workspace
    export ROS_DOMAIN_ID=1
    ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
    ```

Then you can use the joystick to control the robot in the simulation: 
```
LB + A: standup
LB + B: sit down
LB + X: start rl controller
LB + RB: stop all the controllers
```

## In Real Robot

Please note that all the following operations are performed on PC instead of robot's onboard computer. 

> **WARNING**: Operating a real robot can be dangerous. Before proceeding, please ensure:
> - You have a clear, open space free of obstacles and people.
> - The robot is placed on a flat, stable surface.
> - You are familiar with the emergency stop procedures.
> - Someone is ready to catch or support the robot if it falls.

> **DISCLAIMER**: The authors and contributors of this software are not responsible for any damage, injury, or loss caused by the use of this package. Use at your own risk. Always prioritize safety when working with physical robots.


### Hardware Setup

Connect your PC to the robot's onboard computer via Ethernet. Make sure to set the correct IP address and subnet mask on your PC's Ethernet interface to communicate with the robot, refer to [Unitree's documentation](https://support.unitree.com/home/en/developer/Quick_start) for details.

### Bringup the robot

1. **Disable Go2's main motion control service**: You can use the `go2_stand_example` in `unitree_sdk2` to stop the robot's own motion control service:
    ```bash
    # Make sure the robot is lying down before running the command
    cd path/to/unitree_sdk2/build/bin
    ./go2_stand_example 
    ```
2. **Run the controller**: Make sure you have shutdown the above terminal and source the `legged_ros2` workspace in a new terminal, then run the control node:
    ```bash
    source path/to/unitree_ros2/setup.sh # Source ROS2 and change DDS implementation to avoid DDS conflicts
    source ~/legged_ws/install/setup.bash # Source legged_ros2 workspace
    ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
    ```
Then you can use the joystick to control the robot: 
```
L1 + A: standup
L1 + B: sit down
L1 + X: start rl controller
L1 + RB: stop all the controllers
```


## Docker network interface for CycloneDDS

When you use `/root/legged_ws/setup.sh` in Docker, you must specify the network interface for CycloneDDS to work properly. The setup script will determine the network interface as follows:

- If `NET_IF` is set, it uses that interface.
- Otherwise it auto-detects the default route interface (and falls back to `eth0`).

Examples:

```bash
# Default interface
source /root/legged_ws/setup.sh
```

```bash
# Force a specific interface
export NET_IF=eth0
source /root/legged_ws/setup.sh
```
