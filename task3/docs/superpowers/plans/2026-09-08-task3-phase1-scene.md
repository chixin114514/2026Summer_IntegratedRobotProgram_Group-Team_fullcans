# Task3 Phase 1 Scene Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independent Task3 ROS 2 workspace that launches the unchanged MechArm 270 simulation model inside the first tabletop sorting scene with P1-P4, BIN_A/B, two target classes, and an overhead RGB camera.

**Architecture:** Task3 owns its workspace, world, launch, configuration, and tests. A minimal Task3-local `mycobot_description` package preserves the original `package://mycobot_description/...` mesh paths, while the selected MechArm URDF and seven M5 DAE files are copied byte-for-byte from `tak2`; Task3 never imports or launches `task2_sim`.

**Tech Stack:** ROS 2 Humble, Python 3.10, ament_python, Ignition Gazebo Fortress 6.18, `ros_ign_bridge`, SDF 1.8, Python `unittest` and `xml.etree.ElementTree`.

---

## File Map

Create only these Stage 1 files:

```text
ros2_ws/src/mycobot_description/
├── mycobot_description/__init__.py
├── package.xml
├── resource/mycobot_description
├── setup.cfg
├── setup.py
└── urdf/mecharm_270_m5/{base,link1,link2,link3,link4,link5,link6}.dae

ros2_ws/src/task3_sim/
├── config/scene.yaml
├── launch/task3_scene.launch.py
├── package.xml
├── resource/task3_sim
├── setup.cfg
├── setup.py
├── task3_sim/__init__.py
├── test/test_description_assets.py
├── test/test_scene_world.py
├── test/test_scene_launch.py
├── urdf/mecharm_270_gazebo.urdf
└── worlds/task3_world.sdf
```

The source URDF is:

```text
../tak2/ros2_ws/src/task2_sim/urdf/mecharm_270_gazebo.urdf
```

It and all copied DAE files must remain byte-identical. Do not copy Task2 Python, configuration, world, launch, or result logic.

### Task 1: Immutable Robot Assets and Minimal Description Package

**Files:**
- Create: `ros2_ws/src/task3_sim/test/test_description_assets.py`
- Create: `ros2_ws/src/mycobot_description/package.xml`
- Create: `ros2_ws/src/mycobot_description/setup.py`
- Create: `ros2_ws/src/mycobot_description/setup.cfg`
- Create: `ros2_ws/src/mycobot_description/resource/mycobot_description`
- Create: `ros2_ws/src/mycobot_description/mycobot_description/__init__.py`
- Copy unchanged: `ros2_ws/src/task3_sim/urdf/mecharm_270_gazebo.urdf`
- Copy unchanged: seven `ros2_ws/src/mycobot_description/urdf/mecharm_270_m5/*.dae` files

- [ ] **Step 1: Write the failing immutable-asset test**

Create a standard-library `unittest` that locates the repository from `Path(__file__).resolve()`, compares the destination URDF bytes with the exact `tak2` source, and compares these seven mesh files byte-for-byte:

```python
MESH_FILES = [
    'base.dae', 'link1.dae', 'link2.dae', 'link3.dae',
    'link4.dae', 'link5.dae', 'link6.dae',
]
```

The test must assert that every
`package://mycobot_description/urdf/mecharm_270_m5/` reference resolves inside
the Task3-local asset package. The immutable source URDF has historical
`/task2/` Gazebo control topic names; preserve those bytes and restrict the
no-Task2-runtime assertion to Task3-owned world, configuration, launch, and
Python files.

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_description_assets.py -v
```

Expected: FAIL because the Task3 asset packages and copied files do not exist.

- [ ] **Step 3: Create the minimal package and copy assets unchanged**

Use `ament_python` with package name `mycobot_description`. `setup.py` installs only the package marker, `package.xml`, and `urdf/mecharm_270_m5/*.dae`; do not copy the entire vendor model collection. Copy the URDF and meshes without content edits.

- [ ] **Step 4: Run the immutable-asset test and verify GREEN**

Run the same `unittest` command. Expected: all tests PASS.

- [ ] **Step 5: Review scope**

Run:

```bash
find ros2_ws/src/mycobot_description ros2_ws/src/task3_sim/urdf -type f | sort
rg -n "task2_sim|task_points|task_manager|result_monitor" ros2_ws/src
```

Expected: only the listed model assets are present; forbidden Task2 runtime/configuration references are absent.

Do not commit. The root agent will review the diff without mixing the user's existing staged files.

### Task 2: Tabletop Scene and Camera

**Files:**
- Create: `ros2_ws/src/task3_sim/test/test_scene_world.py`
- Create: `ros2_ws/src/task3_sim/worlds/task3_world.sdf`
- Create: `ros2_ws/src/task3_sim/config/scene.yaml`

- [ ] **Step 1: Write the failing world contract test**

Parse the SDF using `xml.etree.ElementTree`. Assert:

```python
REQUIRED_MODELS = {
    'ground_plane', 'table', 'grid_p1', 'grid_p2',
    'grid_p3', 'grid_p4', 'bin_a', 'bin_b',
    'orange_battery_1', 'green_can_1', 'overhead_camera',
}
```

Also assert:

- world name is `task3_world`;
- Physics, UserCommands, SceneBroadcaster, and Sensors systems exist;
- the camera sensor type is `camera`;
- image width/height are `640`/`480`;
- camera topic is `/task3/camera/image_raw`;
- both target models have `<static>false</static>` and inertial/collision elements;
- no `SetEntityPose`, `set_pose`, Task2 A/B point, or Task2 node appears.

- [ ] **Step 2: Run the world test and verify RED**

Run:

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_scene_world.py -v
```

Expected: FAIL because `task3_world.sdf` does not exist.

- [ ] **Step 3: Implement the minimal SDF world**

Use these fixed centres in metres, all on a tabletop whose top is `z=0.400`:

```yaml
arm:  {x:  0.00, y:  0.00}
p1:   {x:  0.16, y:  0.14}
p2:   {x:  0.16, y: -0.14}
p3:   {x: -0.16, y:  0.14}
p4:   {x: -0.16, y: -0.14}
bin_a:{x:  0.16, y:  0.00}
bin_b:{x: -0.16, y:  0.00}
```

Scene requirements:

- table size `0.80 x 0.70 x 0.40 m`;
- four thin, labelled-by-colour, visual-only square grid markers,
  `0.09 x 0.09 m`, with no collision geometry;
- BIN_A and BIN_B as shallow trays with a base and four low walls;
- `orange_battery_1` at P1 as a dynamic orange box, `0.070 x 0.035 x 0.035 m`;
- `green_can_1` at P4 as a dynamic green cylinder, length `0.070 m`, radius `0.0175 m`, rotated horizontal;
- realistic positive mass/inertia and friction for both objects;
- overhead camera pose `0 0 1.35 0 1.5708 0`, update rate `30`, horizontal FOV about `1.05`, `R8G8B8`, near `0.05`, far `5.0`;
- camera Sensors system uses Ogre2.

Put the same coordinates and names in `config/scene.yaml`; this file is Task3-owned and must not contain Task2 A/B semantics.

- [ ] **Step 4: Run the world contract test and verify GREEN**

Run the same `unittest` command. Expected: all tests PASS.

- [ ] **Step 5: Parse both data files**

Run:

```bash
python3 -c "import xml.etree.ElementTree as E; E.parse('ros2_ws/src/task3_sim/worlds/task3_world.sdf'); print('SDF_OK')"
python3 -c "import yaml; d=yaml.safe_load(open('ros2_ws/src/task3_sim/config/scene.yaml')); print(sorted(d['grids']), sorted(d['bins']))"
```

Expected: `SDF_OK`, grids `P1`-`P4`, and bins `BIN_A`/`BIN_B`.

Do not commit.

### Task 3: Task3 Simulation Package and Scene Launch

**Files:**
- Create: `ros2_ws/src/task3_sim/test/test_scene_launch.py`
- Create: `ros2_ws/src/task3_sim/package.xml`
- Create: `ros2_ws/src/task3_sim/setup.py`
- Create: `ros2_ws/src/task3_sim/setup.cfg`
- Create: `ros2_ws/src/task3_sim/resource/task3_sim`
- Create: `ros2_ws/src/task3_sim/task3_sim/__init__.py`
- Create: `ros2_ws/src/task3_sim/launch/task3_scene.launch.py`

- [ ] **Step 1: Write the failing launch contract test**

The test reads the launch source as text and asserts it contains:

- `ign`, `gazebo`, and `-r`;
- `task3_world.sdf` and unchanged `mecharm_270_gazebo.urdf`;
- `robot_state_publisher`;
- `ros_ign_bridge` and `parameter_bridge`;
- `/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock`;
- `/task3/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image`;
- `/world/task3_world/create`;
- robot entity name `mecharm_270` and spawn `z=0.40`.

It also asserts that the launch contains none of:

```text
task2_sim, task_manager, result_monitor, task_points.yaml, /task2/
```

- [ ] **Step 2: Run the launch test and verify RED**

Run:

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_scene_launch.py -v
```

Expected: FAIL because package and launch files do not exist.

- [ ] **Step 3: Create the package and minimal launch**

The `task3_sim` ament_python package installs `launch/*.launch.py`,
`worlds/*.sdf`, `config/*.yaml`, and `urdf/*.urdf`. Declare only Stage 1
runtime dependencies: `launch`, `launch_ros`, `ament_index_python`,
`robot_state_publisher`, `ros_ign_bridge`, `rosgraph_msgs`, `sensor_msgs`,
`mycobot_description`, and `python3-yaml`.

The launch must:

1. extend `IGN_GAZEBO_RESOURCE_PATH` with the installed share parent;
2. run `ign gazebo -r <task3_world.sdf>`;
3. bridge `/clock` and `/task3/camera/image_raw`;
4. publish the unchanged URDF as `robot_description`;
5. call `/world/task3_world/create` after a short delay to spawn `mecharm_270` at `z=0.40`;
6. start no Task2 node, detector, controller, Action, or state machine.

- [ ] **Step 4: Run the launch test and syntax check**

Run:

```bash
python3 -m unittest ros2_ws/src/task3_sim/test/test_scene_launch.py -v
python3 -m py_compile ros2_ws/src/task3_sim/launch/task3_scene.launch.py
```

Expected: tests PASS and `py_compile` exits 0.

- [ ] **Step 5: Run all Stage 1 automated checks**

Run:

```bash
python3 -m unittest discover -s ros2_ws/src/task3_sim/test -p 'test_*.py' -v
```

Expected: all Stage 1 tests PASS. This proves only static contracts, not Gazebo visual correctness.

Do not commit.

### Task 4: Root Review, Jetson Build, and Human Acceptance Handoff

**Files:**
- No new implementation files expected.

- [ ] **Step 1: Root agent reviews the actual diff**

Check every created path, byte-compare immutable assets with `cmp`, inspect
`git diff --stat`, and scan for Task2 runtime/configuration imports. Reject
extra frameworks, controllers, vision nodes, Actions, and state machines in
Stage 1.

- [ ] **Step 2: Synchronize only Task3 to Jetson**

Use checksum-preserving `rsync` to:

```text
nvidia@192.168.55.1:/home/nvidia/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/
```

Exclude local/remote `build/`, `install/`, `log/`, `__pycache__/`, and `.DS_Store` from transfer. Do not modify `tak2`.

- [ ] **Step 3: Build the independent Task3 workspace**

On Jetson:

```bash
cd /home/nvidia/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mycobot_description task3_sim
```

Expected: both packages finish successfully. A successful build is not Gazebo acceptance.

- [ ] **Step 4: Verify installed package discovery**

Run:

```bash
source install/setup.bash
ros2 pkg prefix mycobot_description
ros2 pkg prefix task3_sim
```

Expected: both resolve inside the Task3 workspace install.

- [ ] **Step 5: Issue Human Acceptance 1 instructions**

Provide the user this launch command:

```bash
cd /home/nvidia/Desktop/2026Summer_IntegratedRobotProgram_Group-Team_fullcans/task3/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch task3_sim task3_scene.launch.py
```

Ask the user to observe the actual Gazebo window for robot pose, gripper shape,
table/grid/bin placement, two flat objects, camera placement, clipping,
collisions, and model instability. Do not mark Human Acceptance 1 passed until
the user confirms those observations.
