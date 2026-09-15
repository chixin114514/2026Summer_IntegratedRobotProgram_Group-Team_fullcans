# Task 3: Tabletop object sorting

Task 3 contains ROS 2 packages for fixed-position sorting in Ignition Gazebo and on a MechArm 270. The simulation packages expose a `SortObject` action. The hardware packages either classify the initial six-station layout and follow calibrated command targets, or convert camera detections into table coordinates for vision-guided pickup.

## Packages

| Package | Purpose |
| --- | --- |
| `task3_interfaces` | Defines the `SortObject` action used by the simulation servers and clients |
| `task3_sim` | One-object Gazebo scene and action server |
| `task3_six_sim` | Six-object yellow and green sorting scene |
| `task3_six_fast` | Faster six-object scene with RGB color classification and optional automatic sorting |
| `fixed_joint_sorter` | Real-robot sorter that classifies the initial layout, then uses fixed command targets |
| `task3_vision` | Real-robot sorter using YOLO detections, table calibration, and inverse kinematics |
| `mycobot_description` | Task 3 robot mesh assets |

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- Ignition Gazebo, `ros_ign_bridge`, and `robot_state_publisher` for simulation
- PyYAML
- `pymycobot` for real-robot operation
- A detector that publishes JSON in `std_msgs/msg/String` on `/yolo/detections` for the hardware sorters

## Build

```bash
cd task3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select \
  mycobot_description task3_interfaces task3_sim task3_six_sim \
  task3_six_fast fixed_joint_sorter task3_vision
source install/setup.bash
```

## Six-object simulation

Start the reference scene:

```bash
ros2 launch task3_six_sim task3_six_scene.launch.py headless:=false
```

In another sourced terminal, send all six configured jobs:

```bash
ros2 run task3_six_sim sort_all
```

Set `TASK3_SIX_JOBS` to run selected stations during calibration:

```bash
TASK3_SIX_JOBS=P1,P2 ros2 run task3_six_sim sort_all
```

The reference server writes JSON Lines records to `~/.ros/task3_six_sort_log.jsonl`.

The faster package can read the simulated RGB camera and build the yellow and green sort plan automatically:

```bash
ros2 launch task3_six_fast task3_six_fast_scene.launch.py \
  headless:=false auto_sort:=true
```

Use `headless:=true` on either six-object launch file when no Gazebo window is needed. The fast server logs to `~/.ros/task3_six_fast_sort_log.jsonl`.

## One-object simulation

```bash
ros2 launch task3_sim task3_scene.launch.py
```

The action accepts a pickup grid ID and a destination bin ID:

```text
string grid_id
string bin_id
---
bool success
bool pick_verified
bool place_verified
string object_name
string message
```

This server writes its default log to `~/.ros/task3_sort_log.jsonl`.

## Fixed-joint hardware sorter

`fixed_joint_sorter` reads repeated YOLO frames, assigns the six detections to `P1` through `P6`, checks the expected class count for each bin, and then sends the fixed six-value targets in `config/fixed_sorter.yaml` with `send_coords()`.

First validate detection and routing without moving the arm:

```bash
ros2 run fixed_joint_sorter fixed_joint_sort
```

The default `live` value is `false`. Before a live run, check the robot address, class names, station targets, bin targets, and joint limits in `ros2_ws/src/fixed_joint_sorter/config/fixed_sorter.yaml`. When the values match the connected setup and the work area is clear, enable motion explicitly:

```bash
ros2 run fixed_joint_sorter fixed_joint_sort --ros-args -p live:=true
```

The sorter uses camera results only for the initial layout. It stops consuming detections before robot motion begins.

## Vision-guided hardware sorter

The `task3_vision` path expects a separate camera and YOLO node on `/yolo/detections`. Its configuration is `ros2_ws/src/task3_vision/config/vision_config.yaml`.

1. Check the configured regions and inverse-kinematics workspace without connecting to the robot:

   ```bash
   ros2 run task3_vision check_workspace
   ```

2. Start the detector, then record at least four table calibration points:

   ```bash
   ros2 run task3_vision calibrate_table
   ```

   The default output is `~/.ros/task3_table_calibration.yaml`.

3. Inspect the computed plan without robot motion:

   ```bash
   ros2 launch task3_vision task3_vision.launch.py dry_run:=true
   ```

4. After checking the calibration, class routes, limits, robot connection, and clearances, start the hardware run:

   ```bash
   ros2 launch task3_vision task3_vision.launch.py
   ```

The default run log is `~/.ros/task3_vision_sort.jsonl`. The current implementation records arrival timeouts and command errors, then continues to the next step, so an operator must watch the arm throughout a live run.

## Tests

```bash
cd task3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select \
  task3_interfaces task3_sim task3_six_sim task3_six_fast \
  fixed_joint_sorter task3_vision
colcon test-result --verbose
```
