# How to Add a New Robot to legged_ros2

This guide explains how to integrate a new legged robot into the current `legged_ros2` stack.

> Note: `IO_descriptors.yaml` under `config/rl_policy/` is **auto-exported by Isaac Lab**. Do **not** hand-edit it.

[TOC]

## 1. What You Need to Add

To support a new robot, you usually need updates in these places:

1. `legged_robot_description` (URDF/Xacro + launch + controller YAML)
2. `legged_ros2_control` (hardware/system interface plugin)
3. `legged_rl_controller` / `legged_ros2_controller` config wiring
4. Optional: `legged_mapping` (if your sensor/TF setup differs)

## 2. Create a Robot Description Package

Create a new description package, typically by mirroring `go2_description` structure:

- `urdf/` (robot model, ros2_control tags, transmissions)
- `config/ros2_control/*.yaml` (controller manager + controllers)
- `config/main_loop/*.yaml` (wireless binding / controller switching behavior)
- `launch/bringup_*.launch.py` (RL/static/broadcaster modes)
- `config/rl_policy/` (policy assets)

### Required consistency checks

- Joint names in URDF and ros2_control YAML must match exactly.
- IMU sensor name in ros2_control YAML (e.g. `imu`) must exist in the URDF ros2_control block.
- Controller lists in YAML must match the plugins you actually load.
- For `rl_controller`, `joint_names` order in `config/ros2_control/*.yaml` must match the policy order from Isaac Lab export (`IO_descriptors.yaml`, especially `articulations.robot.joint_names` and action joint order).

## 3. Implement Hardware Interface in `legged_ros2_control`

Add a robot-specific backend under:

- `src/legged_ros2/legged_ros2_control/src/robots/<your_robot>/`

Typical files:

- `<your_robot>_system_interface.cpp`
- `<your_robot>_lowlevel_node.cpp` (if needed)
- `<your_robot>_main_loop.cpp` (optional, if different from Go2 flow)
- `CMakeLists.txt`

### Must-do integration points

1. Add subdirectory in `legged_ros2_control/CMakeLists.txt`.
2. Export plugin class in `legged_ros2_control/legged_ros2_control_plugins.xml`.
3. Ensure your class derives from `legged::LeggedSystemInterface` and exports required joint/IMU interfaces.

## 4. Connect Bringup Launch

Create or adapt bringup launch files (like `bringup_rl.launch.py`) to wire:

- robot description (`robot_description` from xacro)
- ros2_control YAML
- main-loop YAML
- RL model path (`policy.onnx`)
- IO descriptors path (`IO_descriptors.yaml`)

For a new robot, this usually means a dedicated launch in your new description package.

## 5. RL Policy Assets for the New Robot

For RL mode, put policy assets in your robot description package, e.g.:

- `config/rl_policy/policy.onnx`
- `config/rl_policy/IO_descriptors.yaml`

Important:

- `IO_descriptors.yaml` is exported by Isaac Lab together with the ONNX policy.
- Keep ONNX and descriptors from the **same export run**.
- Do not manually edit descriptors unless you are debugging and fully understand consequences.

## 6. Controller-Level Alignment Checklist

Before running RL:

1. `rl_controller.joint_names` order exactly matches RL descriptor order (`IO_descriptors.yaml`).
2. `imu_names` in controller config are valid and available.
3. Joint stiffness/damping/default joint states are consistent between hardware side and descriptors.
4. `cmd_vel` topic and range parameters are configured for your robot.

## 7. Minimal Validation Steps

1. Build:

```bash
colcon build --packages-select legged_ros2_control legged_ros2_controller legged_rl_controller <your_description_pkg>
```

2. Start bringup in broadcaster-only mode first.
3. Check interfaces:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

4. Activate static controller first (safe posture), then RL controller.
5. Verify no joint-order mismatch, action-dimension mismatch, or IMU missing errors.

## 8. Common Failure Modes

- Joint name/order mismatch between hardware and RL descriptors.
- IMU interface not available (controller configuration fails).
- Wrong plugin class name in `legged_ros2_control_plugins.xml`.
- Using ONNX and descriptors from different policy exports.
