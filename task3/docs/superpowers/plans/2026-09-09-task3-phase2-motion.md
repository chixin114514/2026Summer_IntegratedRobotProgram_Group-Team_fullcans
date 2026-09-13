# Task3 Phase 2 Fixed-Point Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute one physically observed fixed-point pickup and classified placement through a ROS 2 Action while keeping the imported MechArm URDF byte-identical.

**Architecture:** Task3 stores reachability-checked joint arrays, publishes smooth joint commands through `/task3/` ROS topics, and uses launch remapping to the immutable URDF's historical Gazebo transport topics. Action success is fail-closed: it requires measured object lift and measured settled placement, not only completed command publication.

**Tech Stack:** ROS 2 Humble, `rclpy`, custom ROS 2 Action, `std_msgs/Float64`, `tf2_msgs/TFMessage`, Ignition Gazebo Fortress, `ros_ign_bridge`, Python 3.10.

---

## File Map

```text
ros2_ws/src/task3_interfaces/
├── action/SortObject.action
├── CMakeLists.txt
└── package.xml

ros2_ws/src/task3_sim/
├── config/motion_points.yaml
├── task3_sim/motion_plan.py
├── task3_sim/pick_sort_server.py
└── test/
    ├── test_motion_config.py
    ├── test_motion_plan.py
    └── test_action_contract.py
```

Modify only Task3-owned `scene.yaml`, `task3_world.sdf`, `task3_scene.launch.py`,
`package.xml`, and `setup.py`. Do not edit the imported URDF or vendor meshes.

### Task 1: Reachable Scene Coordinates and Fixed Joint Poses

**Files:**
- Create: `ros2_ws/src/task3_sim/test/test_motion_config.py`
- Create: `ros2_ws/src/task3_sim/config/motion_points.yaml`
- Modify: `ros2_ws/src/task3_sim/config/scene.yaml`
- Modify: `ros2_ws/src/task3_sim/worlds/task3_world.sdf`
- Modify: `ros2_ws/src/task3_sim/test/test_scene_world.py`

- [ ] **Step 1: Write failing configuration tests**

Assert that SDF and `scene.yaml` use these reachability-checked XY centres:

```python
P1 = [ 0.13213,  0.06812]
P2 = [ 0.08037, -0.12506]
P3 = [-0.08037,  0.12506]
P4 = [-0.13213, -0.06812]
BIN_A = [ 0.12557, -0.03365]
BIN_B = [-0.11782,  0.05494]
```

Assert that `motion_points.yaml` defines exactly six arm joints for every pose,
all values are finite and inside these degree limits:

```yaml
joint_limits_deg:
  lower: [-160, -75, -175, -155, -115, -180]
  upper: [ 160, 120,   65,  155,  115,  180]
```

Require at least `home`, `P1-P4.above`, `P1-P4.pick`, `BIN_A/B.above`, and
`BIN_A/B.place`. Require common pickup and place heights and common gripper
open/closed values.

- [ ] **Step 2: Run RED**

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_motion_config.py -v
```

Expected: FAIL because `motion_points.yaml` is missing and scene coordinates
still use the larger Stage 1 layout.

- [ ] **Step 3: Update scene and add fixed joint configuration**

Use these joint arrays in degrees:

```yaml
home: [0.0, 0.0, -20.0, 0.0, 110.0, 0.0]
picks:
  P1:
    above: [27.2735, 24.7979, -33.2205, 0.0, 98.4168, 27.2735]
    pick:  [27.2738, 28.4655,  -5.3134, 0.0, 66.8504, 27.2738]
  P2:
    above: [-57.2738, 24.7979, -33.2205, 0.0, 98.4168, -57.2738]
    pick:  [-57.2736, 28.4655,  -5.3134, 0.0, 66.8504, -57.2736]
  P3:
    above: [122.7262, 24.7979, -33.2205, 0.0, 98.4168, 122.7262]
    pick:  [122.7264, 28.4655,  -5.3134, 0.0, 66.8504, 122.7264]
  P4:
    above: [-152.7265, 24.7979, -33.2205, 0.0, 98.4168, -152.7265]
    pick:  [-152.7262, 28.4655,  -5.3134, 0.0, 66.8504, -152.7262]
bins:
  BIN_A:
    above: [-15.0001, 13.4886, -21.1744, 0.0, 97.6824, -15.0001]
    place: [-14.9997, 17.7872,   6.7405, 0.0, 65.4799, -14.9997]
  BIN_B:
    above: [154.9999, 13.4886, -21.1744, 0.0, 97.6824, 154.9999]
    place: [155.0003, 17.7872,   6.7405, 0.0, 65.4799, 155.0003]
```

Set `gripper.open_rad=0.14`, `gripper.closed_rad=-0.50`, command rate `20 Hz`,
and conservative durations: home/transfer `2.0 s`, vertical motion `1.2 s`,
gripper `0.8 s`, settle `1.0 s`.

Move the two initial objects with their grids. Preserve table height, camera,
BIN construction, grid sizes, object dimensions, and requested topology.

- [ ] **Step 4: Run GREEN and all scene tests**

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_motion_config.py -v
python3 -m unittest ros2_ws/src/task3_sim/test/test_scene_world.py -v
```

Expected: PASS.

### Task 2: Minimal SortObject Action Interface

**Files:**
- Create: `ros2_ws/src/task3_interfaces/action/SortObject.action`
- Create: `ros2_ws/src/task3_interfaces/CMakeLists.txt`
- Create: `ros2_ws/src/task3_interfaces/package.xml`
- Create: `ros2_ws/src/task3_sim/test/test_action_contract.py`

- [ ] **Step 1: Write the failing Action contract test**

Require this exact semantic contract:

```text
string grid_id
string bin_id
---
bool success
bool pick_verified
bool place_verified
string object_name
string message
---
string stage
float32 progress
```

Require `rosidl_default_generators`, `action_msgs`, and
`rosidl_default_runtime` in the interface package.

- [ ] **Step 2: Run RED**

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_action_contract.py -v
```

Expected: FAIL because `task3_interfaces` does not exist.

- [ ] **Step 3: Create the three interface files**

Use a minimal `ament_cmake` interface package and generate only
`action/SortObject.action`. Do not add services or messages.

- [ ] **Step 4: Run GREEN**

Run the same test. Expected: PASS.

### Task 3: Small Pure Motion Plan Helper

**Files:**
- Create: `ros2_ws/src/task3_sim/task3_sim/motion_plan.py`
- Create: `ros2_ws/src/task3_sim/test/test_motion_plan.py`

- [ ] **Step 1: Write failing behavior tests**

Test only three public behaviors without ROS mocks:

1. `validate_goal('P1', 'BIN_A', config)` accepts known identifiers.
2. Invalid grid/bin raises `ValueError` before any command is published.
3. `build_sequence('P1', 'BIN_A', config)` returns this ordered sequence:

```text
OPEN, PICK_ABOVE, PICK, CLOSE, LIFT, BIN_ABOVE,
BIN_PLACE, RELEASE, RETREAT, HOME
```

The returned arm poses must each contain six joint values and the sequence must
use only the requested grid and bin entries.

- [ ] **Step 2: Run RED**

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_motion_plan.py -v
```

Expected: FAIL because `motion_plan.py` is missing.

- [ ] **Step 3: Implement only the tested helper**

Use a small immutable `MotionStep` dataclass with fields `stage`,
`arm_deg`, `gripper_rad`, and `duration_s`. Do not add a generic planning
framework, plugin interface, or online IK.

- [ ] **Step 4: Run GREEN**

Run the same test. Expected: PASS.

### Task 4: Action Server, Gazebo Topic Adapter, and Pose Evidence

**Files:**
- Create: `ros2_ws/src/task3_sim/task3_sim/pick_sort_server.py`
- Modify: `ros2_ws/src/task3_sim/launch/task3_scene.launch.py`
- Modify: `ros2_ws/src/task3_sim/package.xml`
- Modify: `ros2_ws/src/task3_sim/setup.py`
- Modify: `ros2_ws/src/task3_sim/test/test_scene_launch.py`
- Modify: `ros2_ws/src/task3_sim/test/test_description_assets.py`

- [ ] **Step 1: Extend launch contracts and run RED**

Require the launch to:

- bridge six arm and six gripper command topics from ROS to Gazebo;
- expose ROS-side names under `/task3/arm/` and `/task3/gripper/` using ROS
  remappings while preserving the immutable Gazebo `/task2/...` names;
- bridge `/world/task3_world/pose/info` from Gazebo to
  `tf2_msgs/msg/TFMessage`;
- start `pick_sort_server` with `use_sim_time=true` and both Task3 YAML files;
- depend on `rclpy`, `std_msgs`, `tf2_msgs`, and `task3_interfaces`.

Update the isolation test so the only permitted `/task2/` strings outside the
immutable URDF are explicit Gazebo-side entries paired one-for-one with a
`/task3/` ROS remapping. Task3 must still contain no Task2 package, node,
configuration, or Python import.

- [ ] **Step 2: Implement the Action server**

Use `MultiThreadedExecutor` and a reentrant callback group. The server must:

1. reject concurrent goals and unknown identifiers;
2. publish all arm poses by smoothstep interpolation at 20 Hz;
3. publish all six linkage joint values for open/close;
4. publish Action feedback for every stage;
5. select the nearest `orange_battery_*` or `green_can_*` model pose within
   `0.06 m` of the requested grid;
6. record pose samples while executing;
7. after LIFT require measured object rise of at least `0.025 m`;
8. after RELEASE/HOME require object XY within `0.045 m` of the requested BIN,
   object centre Z in `[0.410, 0.440] m`, and estimated speed no more than
   `0.03 m/s`;
9. on invalid goal, missing pose, failed lift, failed placement, or cancel:
   open the gripper, command HOME, and return a non-success result;
10. never call SetEntityPose or write the target object's pose.

Publish initial HOME/open commands for several timer ticks after startup so a
late bridge subscription still receives them. Do not report readiness until
this initialization finishes.

- [ ] **Step 3: Add only the required package entry point and dependencies**

Add:

```text
pick_sort_server = task3_sim.pick_sort_server:main
```

Install `motion_points.yaml` through the existing config glob.

- [ ] **Step 4: Run local static verification**

```bash
python3 -m unittest discover -s ros2_ws/src/task3_sim/test -p 'test_*.py' -v
python3 -m py_compile ros2_ws/src/task3_sim/task3_sim/motion_plan.py ros2_ws/src/task3_sim/task3_sim/pick_sort_server.py ros2_ws/src/task3_sim/launch/task3_scene.launch.py
```

Expected: all tests pass and Python compilation exits 0. This does not prove
physical grasping.

### Task 5: Review, Jetson Integration, and Human Acceptance 2

**Files:**
- No additional production files expected.

- [ ] **Step 1: Root reviews actual files and both reviewer reports**

Check immutable assets, diff scope, Action contract, joint-limit margins,
Task3/Task2 runtime separation, fail-closed pose checks, and absence of pose
teleportation or unrequested frameworks.

- [ ] **Step 2: Sync only `task3/ros2_ws` to Jetson**

Target:

```text
nvidia@192.168.31.146:/home/nvidia/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws/
```

Do not transfer or delete `build/`, `install/`, `log/`, and do not modify
`tak2`.

- [ ] **Step 3: Clean-build the three packages with the verified layout**

```bash
cd /home/nvidia/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install --merge-install \
  --packages-select mycobot_description task3_interfaces task3_sim
source install/setup.bash
```

Verify all three packages with `ros2 pkg prefix` and verify
`ros2 action list -t` only after launch.

- [ ] **Step 4: Reach Human Acceptance 2**

Launch the scene, then send one explicit calibration goal:

```bash
ros2 action send_goal /task3/sort_object \
  task3_interfaces/action/SortObject \
  "{grid_id: P1, bin_id: BIN_A}" --feedback
```

Ask the user to observe the actual approach, descent, gripper closure, physical
object lift, collision-free transfer, release into BIN_A, retreat, and HOME.
Controller result and pose checks are diagnostic only. Do not mark Human
Acceptance 2 passed until the user confirms the visible Gazebo behavior.
