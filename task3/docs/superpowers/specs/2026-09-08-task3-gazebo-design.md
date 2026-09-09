# Task3 Gazebo Simulation Design

## 1. Scope

Task3 is an independent ROS 2 workspace for automatic tabletop object sorting.
It must not depend on Task2 configuration, task control, result evaluation, or
runtime nodes.

Task2 may only provide selected reusable assets:

- unchanged MechArm 270 URDF files;
- required mesh files;
- small, task-independent kinematics or matrix calculations after review.

The following Task2 files and their logic are explicitly out of scope:

- `task_points.yaml`;
- `task_manager.py`;
- `result_monitor.py`;
- Task2 A/B points, state machine, trial management, and success criteria.

No source file matching `mecharm_270*.urdf` will be edited. If Task3 needs
additional world elements or sensors, they will be defined in Task3-owned
files.

## 2. Environment

The target environment is Ubuntu 22.04.5 LTS on aarch64, ROS 2 Humble,
Python 3.10, and Ignition Gazebo Fortress / Gazebo Sim 6.18.0. The simulator
entry point is `ign gazebo`, and ROS integration uses the installed Ignition
or Gazebo bridge compatible with this environment.

## 3. Minimal Package Structure

Task3 will contain only the packages needed by the assignment:

```text
task3/ros2_ws/src/
├── task3_interfaces/       # one sort-object Action definition
├── mycobot_description/    # Task3-local copy of required vendor assets
└── task3_sim/
    ├── config/             # P1-P4, BIN_A/B and camera mapping
    ├── launch/             # complete simulation launch
    ├── worlds/             # table, grids, bins, objects, camera
    └── task3_sim/
        ├── vision_detector.py
        ├── grid_mapper.py
        ├── pick_sort_server.py
        ├── task_controller.py
        └── task_logger.py
```

This structure is a boundary, not a requirement to add abstraction layers.
Files may be combined where that keeps the implementation smaller and clearer.
The Task3-local `mycobot_description` package keeps the original package URI
used by the immutable URDF while avoiding any runtime dependency on `tak2`.

## 4. Scene

The Task3 world contains:

- one MechArm 270 with gripper;
- one table;
- four fixed pickup grids P1-P4;
- BIN_A for `orange_battery`;
- BIN_B for `green_can`;
- at least one model for each target class;
- one fixed overhead RGB camera.

Both target types are placed flat. Pickup height and gripper opening are shared
between classes. Pickup motion depends only on P1-P4; placement depends only on
the detected class.

## 5. Perception

The Gazebo camera publishes an image bridged to `sensor_msgs/Image`.
The detector processes image pixels and publishes
`vision_msgs/Detection2DArray` containing class, bounding box, and confidence.
It must not manufacture detections from Gazebo ground-truth object poses.

For the first simulation version, simple colour and contour detection is
sufficient. A separate mapper assigns the bounding-box centre to P1-P4 using
the configured image regions.

## 6. Motion and Task Control

A single-object ROS 2 Action accepts the selected pickup grid and destination
bin. It publishes meaningful stage feedback and returns separate pickup and
placement results.

The controller uses a small explicit state machine:

```text
SCAN -> SELECT -> PICK_AND_PLACE -> RETURN_SAFE -> SCAN
```

Only one target is processed at a time. After every attempt, the system returns
to a safe position and requests a fresh detection. `orange_battery` maps to
BIN_A and `green_can` maps to BIN_B.

The initial required exceptions are empty grid and unknown object. An
unreachable goal or grasp failure must return safe or enter a safe stop.

## 7. Logging

Task3-owned logs record:

- detection class, confidence, bounding box, and grid;
- pickup result;
- placement result;
- state transitions;
- exceptions and safe-stop reasons.

Logs support diagnosis but are not evidence that Gazebo motion passed visual
acceptance.

## 8. Verification Boundaries

Automated checks may validate package structure, XML/SDF syntax, configuration,
message fields, mapping calculations, and state transitions. They cannot prove
that the Gazebo model spawned correctly, that the gripper physically held an
object, or that a placement was collision-free.

Four human Gazebo checkpoints are planned:

1. scene, robot, gripper, camera, grids, and bins;
2. one-object physical pickup and placement;
3. image detection, bounding boxes, classes, and grid mapping;
4. multi-object loop, two exception types, and six-object final coverage.

Each checkpoint remains unpassed until the user reports direct visual
observation of the required Gazebo behaviour.

## 9. Implementation Order

1. Create the independent workspace and first scene.
2. Implement one-object fixed-point motion and Action feedback.
3. Add image detection and grid mapping.
4. Connect the single-object state-machine loop.
5. Add repeated processing, exceptions, logs, and unified launch.
6. Run the final two-round, six-object human-observed acceptance sequence.
