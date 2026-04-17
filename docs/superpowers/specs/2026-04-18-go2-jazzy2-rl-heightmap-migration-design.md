# Go2 Jazzy2 RL Heightmap Migration Design

## Summary

Migrate the Go2 RL controller chain in the current `jazzy2` workspace to match the known-good runnable copy at `/home/xcj/work/IsaacLab/legged_ws (Copy)/src/legged_ros2`, while preserving the current branch's old implementation as commented reference blocks.

The migration must stay minimal: change only the code and configuration required to make the RL chain behave like the runnable copy, especially restoring the missing heightmap observation path.

## Goals

- Align the Go2 RL chain with the runnable copy.
- Add the missing heightmap-based observation path.
- Keep old `jazzy2` code in-place as comments for manual comparison and rollback.
- Mark preserved old code and migrated code with the user-requested tags.
- Keep the final patch as small as possible while still making the feature work.

## Non-Goals

- No changes to G1.
- No unrelated refactors.
- No cleanup of existing style outside the touched RL path.
- No changes to wireless controller bindings or unrelated main-loop behavior.

## Source Of Truth

The runnable copy under `/home/xcj/work/IsaacLab/legged_ws (Copy)/src/legged_ros2` is the source of truth for the Go2 RL path.

For this task, "align with the copy" includes both:

- heightmap support
- the copy's Jazzy-side RL launch and observation configuration behavior

## Required Behavior Changes

### 1. Heightmap subscription

The RL controller must subscribe to `unitree_go::msg::HeightMap` on `/heightmap` and store the latest message in a realtime-safe buffer.

### 2. Height scan observation path

The articulation data model must expose a `height_scan` container.

The RL update path must copy heightmap data into `robot_->data.height_scan` before the environment step.

If no heightmap message is available, the RL path must remain operational by falling back to an empty scan at the data level and zeros at the observation level.

### 3. Height scan observation term

The observation registry must include a `height_scan` term.

That term must:

- read `env->robot->data.height_scan`
- use `params.expected_dim`
- return zeros if the source data is absent or size-mismatched

### 4. Explicit observation registration safeguard

Keep the runnable copy's explicit runtime registration safeguard for `height_scan` in `legged_rl_controller.cpp`.

Reason: the runnable copy already protects against shared-library/pluginlib registration issues. Since the goal is behavioral alignment with the known-good version, the current branch should preserve that safeguard rather than replacing it with a larger refactor.

### 5. IO descriptor alignment

`legged_robot_description/go2_description/config/rl_policy/IO_descriptors.yaml` must be aligned with the runnable copy for the Go2 RL policy.

This includes:

- changing existing observation `history_length` values from `5` to `0`
- adding the `height_scan` observation
- keeping `height_scan.params.expected_dim` at `187`

The ONNX input contract must remain consistent with the runnable copy.

### 6. Jazzy launch alignment

`legged_robot_description/go2_description/launch/bringup_rl.launch.py` must adopt the runnable copy's Jazzy-friendly controller spawner behavior.

This includes:

- passing `-p controller_config_path` to spawner nodes
- passing RL controller ROS args through `--controller-ros-args`

Reason: this is part of the copy's effective Jazzy RL startup path and may be required for correct parameter delivery.

### 7. Build dependency alignment

`legged_rl_controller` must declare the `unitree_go` dependency in both:

- `CMakeLists.txt`
- `package.xml`

## Files In Scope

- `legged_rl_controller/include/legged_rl_controller/legged_rl_controller.hpp`
- `legged_rl_controller/src/legged_rl_controller.cpp`
- `legged_rl_controller/include/legged_rl_controller/isaaclab/envs/mdp/observations/observations.h`
- `legged_rl_controller/include/legged_rl_controller/isaaclab/assets/articulation/articulation.h`
- `legged_rl_controller/CMakeLists.txt`
- `legged_rl_controller/package.xml`
- `legged_robot_description/go2_description/config/rl_policy/IO_descriptors.yaml`
- `legged_robot_description/go2_description/launch/bringup_rl.launch.py`

## Files Explicitly Out Of Scope

- `legged_robot_description/go2_description/config/main_loop/rl.yaml`
  It already matches the runnable copy in the inspected version.
- all G1 files
- unrelated ROS 2 control or controller files

## Annotation Strategy

Old code must be preserved as comments using language-valid comment syntax, while keeping the user-requested marker text visible.

Examples:

### C++

```cpp
// <!-- #########jazzy########## -->
// old code
// <!-- #########jazzy########## -->

// <!-- #########new########## -->
// new code, using jazzy version
// <!-- #########new########## -->
```

### Python or YAML

```python
# <!-- #########jazzy########## -->
# old code
# <!-- #########jazzy########## -->

# <!-- #########new########## -->
# new code
# <!-- #########new########## -->
```

Constraints:

- The file must remain syntactically valid.
- Preserve only the directly replaced old logic.
- Do not duplicate large untouched sections just to add comments.

## Minimal-Change Rule

The implementation should prefer the smallest patch that satisfies the required behavior.

That means:

- copy only the RL-chain differences that matter
- avoid introducing abstractions not required for this migration
- do not refactor existing code just for cleanliness
- keep old code comments short and local to replaced logic

## Verification Plan

At minimum, verify:

1. `legged_rl_controller` still builds with the added dependency.
2. The launch file remains syntactically valid.
3. The RL controller can initialize with the updated IO descriptor contract.
4. Missing heightmap messages do not crash the controller because `height_scan` falls back to zeros.

Preferred commands from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_rl_controller go2_description
```

If local Jazzy-specific setup is required in this environment, the verification command may need to be adjusted to the local equivalent before claiming success.

## Risks

### ONNX input mismatch

If the policy file in the current workspace does not match the runnable copy's IO descriptor layout, the controller may fail at runtime even after the code migration.

Mitigation:

- keep the IO descriptor aligned to the runnable copy
- do not change `height_scan.expected_dim`

### Topic contract mismatch

If `/heightmap` is not published with the expected `unitree_go::msg::HeightMap` payload, the controller will fall back to zeros instead of failing hard.

### Comment noise

Preserving old code as comments can make touched files noisier.

Mitigation:

- keep preserved blocks tight
- annotate only replaced logic

## Implementation Direction

After spec approval, implementation should proceed by:

1. adding the data-path and dependency changes
2. aligning the observation and IO descriptor contract
3. aligning the Jazzy launch behavior
4. building the touched packages

