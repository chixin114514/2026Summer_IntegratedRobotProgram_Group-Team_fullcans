# Task 2: Fixed-point pick and place

Task 2 moves a MechArm 270 between a fixed pickup point and a fixed drop point. The same ROS 2 state machine supports Ignition Gazebo, normal real-robot operation, and a real-robot diagnostic mode. It does not use camera-based object localization.

The formal motion sequence is:

`HOME -> OPEN_GRIPPER_INITIAL -> A_SAFE -> A_PREGRASP -> A_PICK -> CLOSE_GRIPPER -> A_LIFT -> B_SAFE -> B_PLACE -> RELEASE_GRIPPER_PARTIAL -> OPEN_GRIPPER -> B_LIFT -> RETURN_HOME`

## Requirements

- Ubuntu 22.04 and ROS 2 Humble
- Ignition Gazebo and `ros_ign_bridge` for simulation
- `robot_state_publisher` and PyYAML
- `pymycobot` for real-robot modes
- A MechArm 270 Pi reachable through the socket or serial settings in `device.yaml` for hardware runs

## Workspace layout

- `ros2_ws/src/task2_sim/launch/task2.launch.py`: formal launch file
- `ros2_ws/src/task2_sim/config/device.yaml`: simulation or hardware mode and connection settings
- `ros2_ws/src/task2_sim/config/task_points.yaml`: HOME, pickup, drop, timing, and acceptance settings
- `ros2_ws/src/task2_sim/config/safety.yaml`: joint limits and safety checks
- `ros2_ws/src/task2_sim/task2_sim/task_manager.py`: task state machine
- `ros2_ws/src/task2_sim/launch/README.md`: list of legacy launch files that are excluded from acceptance runs

## Build

```bash
cd tak2/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select mycobot_description task2_sim
source install/setup.bash
```

## Select a mode

Set `mode` in `ros2_ws/src/task2_sim/config/device.yaml` before building:

| Mode | Operation |
| --- | --- |
| `0` | Ignition Gazebo simulation with automatic acceptance trials |
| `1` | Real robot with automatic acceptance trials |
| `2` | Real robot diagnostic mode for one named state at a time |

The checked-in configuration currently uses mode `2`. Review the robot IP, port or serial device, speed, calibrated waypoints, and safety limits before starting either hardware mode. Keep the arm area clear and make sure an emergency stop is available.

## Run the formal workflow

```bash
ros2 launch task2_sim task2.launch.py
```

Modes `0` and `1` start the experiment manager after the other nodes are ready. The configured acceptance run performs five trials and requires four successful trials.

Mode `2` does not start the experiment manager. Send one existing state name through the diagnostic topic after the launch is ready:

```bash
ros2 topic pub --once /task2/debug_state_request \
  std_msgs/msg/String "{data: 'HOME'}"
```

Valid names are the states shown in the formal motion sequence. The node publishes `DEBUG_COMPLETED: <STATE>` after the requested state finishes. `/task2/task_start` is rejected in mode `2`.

## Results

Each launch creates timestamped files under `~/task2_results/`:

- `trajectory_<timestamp>.csv`
- `task_results_<timestamp>.csv`
- `errors_<timestamp>.log`

The simulation result monitor uses measured object pose and joint feedback. Real-robot motion states wait for fresh measured joint feedback before the next target is sent. Gripper completion remains time based because the current hardware path has no gripper position feedback.

## Tests

```bash
cd tak2/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select task2_sim
colcon test-result --verbose
```

Use `task2.launch.py` for acceptance work. `task2_sim.launch.py`, `task2_auto.launch.py`, and `task2_tune.launch.py` remain in the package for older experiments and tuning.
