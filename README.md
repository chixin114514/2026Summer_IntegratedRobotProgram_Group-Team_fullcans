# 2026 Summer Integrated Robot Program

This repository contains the group project's ROS 2 software for fixed-point manipulation and tabletop sorting with a MechArm 270. Task 2 covers a fixed pickup-and-place cycle. Task 3 adds simulated and camera-assisted object sorting.

The two tasks use separate ROS 2 workspaces. Build and source the workspace for the task you want to run.

## Repository layout

| Directory | Contents | Documentation |
| --- | --- | --- |
| `tak2/` | Fixed-point pickup at A and placement at B in simulation or on a real robot | [Task 2 README](tak2/README.md) |
| `task3/` | One-object and six-object sorting simulations, fixed-target hardware sorting, and vision-guided hardware sorting | [Task 3 README](task3/README.md) |

The `tak2` directory name is retained because it is the current path in the repository.

## Platform

The project targets:

- Ubuntu 22.04
- ROS 2 Humble
- Ignition Gazebo with `ros_ign_bridge`
- MechArm 270 Pi
- Jetson hardware for simulation and real-robot execution

Real-robot paths also require `pymycobot`. The Task 3 hardware sorters expect a detector that publishes JSON detections through ROS 2.

## Quick start

Clone the repository and choose one task workspace.

### Task 2

```bash
cd tak2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select mycobot_description task2_sim
source install/setup.bash
ros2 launch task2_sim task2.launch.py
```

Task 2 selects simulation, normal hardware operation, or single-state hardware diagnosis through `tak2/ros2_ws/src/task2_sim/config/device.yaml`. The checked-in value is currently mode `2`, which connects to a real robot in diagnostic mode. Change and verify the configuration before launching if you intend to use Gazebo.

### Task 3 six-object simulation

```bash
cd task3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select \
  mycobot_description task3_interfaces task3_sim task3_six_sim task3_six_fast
source install/setup.bash
ros2 launch task3_six_fast task3_six_fast_scene.launch.py \
  headless:=false auto_sort:=true
```

The Task 3 workspace also contains two real-robot implementations:

- `fixed_joint_sorter` classifies the initial six-position layout, then sends calibrated fixed targets.
- `task3_vision` maps camera detections to calibrated table coordinates and solves the arm pose for each pickup.

See the [Task 3 README](task3/README.md) for build commands, dry runs, table calibration, log paths, and the reference simulation.

## Safety

Simulation commands do not authorize a hardware run. Before starting any real-robot node:

1. Confirm the selected mode and the robot IP, port, or serial device.
2. Check the calibrated targets, coordinate frame, joint limits, and speed settings.
3. Clear the work area and keep an emergency stop available.
4. Run the available dry-run or single-state diagnostic path before a full sequence.

Task 3 hardware commands that can move the arm are documented separately so they are not copied accidentally from this overview.

## Tests

Run tests from the matching workspace after building it.

Task 2:

```bash
cd tak2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select task2_sim
colcon test-result --verbose
```

Task 3:

```bash
cd task3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select \
  task3_interfaces task3_sim task3_six_sim task3_six_fast \
  fixed_joint_sorter task3_vision
colcon test-result --verbose
```

## Configuration and output

Task-specific YAML files live under each package's `config/` directory. Do not treat similarly named Task 2 and Task 3 files as interchangeable.

Runtime logs and generated results are written outside the source tree, mainly under `~/task2_results/` and `~/.ros/`. The task READMEs list the exact files created by each workflow.
