# Go2 Jazzy2 RL Heightmap Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align the Go2 `jazzy2` RL chain with the runnable copy by adding the heightmap observation path, matching the copy's IO descriptor and Jazzy launch behavior, while preserving replaced code as commented reference blocks.

**Architecture:** Keep the implementation local to the Go2 RL path. Add a realtime `/heightmap` subscription in `legged_rl_controller`, expose the data through `ArticulationData::height_scan`, align `IO_descriptors.yaml` with the runnable copy, and update the Jazzy launch spawner arguments. Preserve replaced logic with local `jazzy` comment blocks and annotate migrated logic with `new` blocks.

**Tech Stack:** ROS 2 controller plugin C++, `yaml-cpp`, ROS 2 launch Python, `unitree_go` messages, `colcon`

---

### Task 1: Migrate The RL Heightmap Data Path

**Files:**
- Modify: `legged_rl_controller/include/legged_rl_controller/legged_rl_controller.hpp`
- Modify: `legged_rl_controller/src/legged_rl_controller.cpp`
- Modify: `legged_rl_controller/include/legged_rl_controller/isaaclab/assets/articulation/articulation.h`
- Modify: `legged_rl_controller/CMakeLists.txt`
- Modify: `legged_rl_controller/package.xml`

- [ ] **Step 1: Make the current RL package fail to build with the future dependency missing**

Expected failing change target:

```cpp
#include "unitree_go/msg/height_map.hpp"
```

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/xcj/work/IsaacLab/legged_ws
colcon build --packages-select legged_rl_controller
```

Expected: build failure because the code has not yet been updated consistently for the new heightmap message path.

- [ ] **Step 2: Add the minimal header and buffer declarations**

Add the new member block in `legged_rl_controller.hpp` near the existing `cmd_vel` subscription:

```cpp
// <!-- #########new########## -->
#include "unitree_go/msg/height_map.hpp"
// <!-- #########new########## -->
```

```cpp
// <!-- #########new########## -->
rclcpp::Subscription<unitree_go::msg::HeightMap>::SharedPtr heightmap_sub_;
using HeightMapMsgSharedPtr = std::shared_ptr<unitree_go::msg::HeightMap>;
using HeightMapBuffer = realtime_tools::RealtimeBuffer<HeightMapMsgSharedPtr>;
std::shared_ptr<HeightMapBuffer> heightmap_buffer_;
// <!-- #########new########## -->
```

Preserve the old area as a tight local comment block if lines are replaced.

- [ ] **Step 3: Add the articulation data field**

Add a minimal field in `articulation.h`:

```cpp
// <!-- #########new########## -->
std::vector<float> height_scan;
// <!-- #########new########## -->
```

- [ ] **Step 4: Implement the heightmap subscription and runtime copy**

In `legged_rl_controller.cpp`, add the runnable-copy behavior:

```cpp
heightmap_buffer_ = std::make_shared<HeightMapBuffer>();
heightmap_sub_ = get_node()->create_subscription<unitree_go::msg::HeightMap>(
  "/heightmap", rclcpp::SystemDefaultsQoS(),
  [this](const unitree_go::msg::HeightMap::SharedPtr msg) {
    heightmap_buffer_->writeFromNonRT(msg);
  });
```

In `on_activate()`:

```cpp
if (heightmap_buffer_) {
  heightmap_buffer_->reset();
}
```

In `update()` before `env_->step();`:

```cpp
auto heightmap_msg = *heightmap_buffer_->readFromRT();
if (heightmap_msg) {
  robot_->data.height_scan.assign(
    heightmap_msg->data.begin(), heightmap_msg->data.end());
} else {
  robot_->data.height_scan.clear();
}
```

- [ ] **Step 5: Align build dependencies**

In `CMakeLists.txt` add:

```cmake
find_package(unitree_go REQUIRED)
```

and include it in:

```cmake
ament_target_dependencies(
  legged_rl_controller PUBLIC
  ...
  unitree_go
)
```

In `package.xml` add:

```xml
<depend>unitree_go</depend>
```

- [ ] **Step 6: Run the package build**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/xcj/work/IsaacLab/legged_ws
colcon build --packages-select legged_rl_controller
```

Expected: `legged_rl_controller` builds successfully.

### Task 2: Align Observations And IO Descriptors

**Files:**
- Modify: `legged_rl_controller/include/legged_rl_controller/isaaclab/envs/mdp/observations/observations.h`
- Modify: `legged_rl_controller/src/legged_rl_controller.cpp`
- Modify: `legged_robot_description/go2_description/config/rl_policy/IO_descriptors.yaml`

- [ ] **Step 1: Add a failing config-contract check target**

Run:

```bash
sed -n '1,260p' /home/xcj/work/IsaacLab/legged_ws/src/legged_ros2/legged_robot_description/go2_description/config/rl_policy/IO_descriptors.yaml
```

Expected: the current file still lacks the runnable copy's `height_scan` observation contract and still shows `history_length: 5` values.

- [ ] **Step 2: Add the minimal `height_scan` observation term**

Add to `observations.h`:

```cpp
REGISTER_OBSERVATION(height_scan)
{
  size_t expected_dim = 0;
  if (params["params"]["expected_dim"]) {
    expected_dim = params["params"]["expected_dim"].as<size_t>();
  }

  const auto & source = env->robot->data.height_scan;
  if (!source.empty() && source.size() == expected_dim) {
    return source;
  }

  return std::vector<float>(expected_dim, 0.0f);
}
```

Keep the replaced section commented locally using the requested markers.

- [ ] **Step 3: Add the explicit runtime registration safeguard**

Add near the top of `legged_rl_controller.cpp`:

```cpp
namespace {
void ensure_observations_registered()
{
  auto & m = isaaclab::observations_map();
  if (m.find("height_scan") == m.end()) {
    m["height_scan"] = [](isaaclab::ManagerBasedRLEnv * env, YAML::Node params)
      -> std::vector<float> {
      size_t expected_dim = 0;
      if (params["params"]["expected_dim"]) {
        expected_dim = params["params"]["expected_dim"].as<size_t>();
      }
      const auto & source = env->robot->data.height_scan;
      if (!source.empty() && source.size() == expected_dim) {
        return source;
      }
      return std::vector<float>(expected_dim, 0.0f);
    };
  }
}
}  // namespace
```

Call it before constructing `ManagerBasedRLEnv`:

```cpp
ensure_observations_registered();
```

- [ ] **Step 4: Align `IO_descriptors.yaml` with the runnable copy**

Preserve replaced config as commented `jazzy` blocks and migrate the active config to:

```yaml
  - dtype: torch.float32
    extras:
      description: Height scan from heightmap around the robot.
      modifiers: null
    full_path: isaaclab.envs.mdp.observations.height_scan
    mdp_type: Observation
    name: height_scan
    observation_type: HeightScan
    overloads:
      clip:
      - -1.0
      - 1.0
      flatten_history_dim: true
      history_length: 0
      scale: null
    params:
      expected_dim: 187
    shape:
    - 187
```

Also align the existing observation `history_length` entries that changed in the runnable copy from `5` to `0`.

- [ ] **Step 5: Re-read the active descriptor section**

Run:

```bash
sed -n '120,360p' /home/xcj/work/IsaacLab/legged_ws/src/legged_ros2/legged_robot_description/go2_description/config/rl_policy/IO_descriptors.yaml
```

Expected: active config contains `height_scan` and the aligned `history_length: 0` values.

### Task 3: Align Jazzy Launch Wiring And Verify The Minimal Path

**Files:**
- Modify: `legged_robot_description/go2_description/launch/bringup_rl.launch.py`

- [ ] **Step 1: Confirm the current launch file still uses the older spawner argument style**

Run:

```bash
sed -n '1,260p' /home/xcj/work/IsaacLab/legged_ws/src/legged_ros2/legged_robot_description/go2_description/launch/bringup_rl.launch.py
```

Expected: spawners do not yet pass the runnable copy's `-p controller_config_path` and RL `--controller-ros-args` pattern.

- [ ] **Step 2: Apply the minimal Jazzy launch alignment**

Add the runnable-copy launch argument block:

```python
rl_controller_ros_args = [
    "--ros-args -p onnx_model_path:=",
    onnx_model_path,
    " -p io_descriptors_path:=",
    io_descriptors_path,
]
```

Update spawner arguments so the active code matches the runnable copy pattern:

```python
arguments=[
    "rl_controller",
    "-c",
    "/controller_manager",
    "--inactive",
    "-p",
    controller_config_path,
    "--controller-ros-args",
    rl_controller_ros_args,
]
```

Also add `-p controller_config_path` to the other spawners that changed in the runnable copy. Preserve the old active lines as commented `jazzy` blocks.

- [ ] **Step 3: Verify the Python file is syntactically valid**

Run:

```bash
python3 -m py_compile /home/xcj/work/IsaacLab/legged_ws/src/legged_ros2/legged_robot_description/go2_description/launch/bringup_rl.launch.py
```

Expected: command exits successfully with no output.

- [ ] **Step 4: Run the focused package build**

Run:

```bash
source /opt/ros/humble/setup.bash
cd /home/xcj/work/IsaacLab/legged_ws
colcon build --packages-select legged_rl_controller go2_description
```

Expected: both touched packages build successfully.

- [ ] **Step 5: Record any remaining runtime-only risk**

If build passes but live `/heightmap` publishing is unavailable in this environment, note that runtime topic validation remains pending even though the code path, launch wiring, and descriptor contract have been aligned.

