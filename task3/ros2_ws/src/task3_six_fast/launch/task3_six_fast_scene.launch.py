"""Launch the faster six-grid sort.

The scene, world, robot description, bridge and action namespace are exactly the
ones ``task3_six_sim/task3_six_scene.launch.py`` uses, so the calibrated geometry
stays valid and the existing ``sort_all`` client works unchanged.  Nothing that
already existed is modified: the world and the shared URDF are read from the
packages that own them.

    ros2 launch task3_six_fast task3_six_fast_scene.launch.py headless:=false

The motion configuration is built here from the reference one at launch time,
with only the timing blocks replaced.  Deriving it instead of storing a second
copy of the file means the poses cannot drift apart from the ones the geometry
was calibrated against.

The speed comes from letting transit steps stop early, not from moving faster.
See ``fast_pick_sort_server.py`` for what changes and why simply raising the trim
gain does not work.
"""
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Faster than the reference, except for the descent segments.  The arm joints run
# soft loops, so a faster descent increases the tracking lag exactly while the
# open jaws are sliding past a block, and that is what knocks blocks over.  The
# descent keeps its 4.0 s.
FAST_DURATIONS_S = {
    "home": 1.5,
    "transfer": 1.8,
    "vertical": 1.8,
    "gripper": 1.0,
    "settle": 1.0,
    "descent_segment": 4.0,
}
FAST_CLOSED_LOOP = {
    "settle_velocity_rad_s": 0.03,
    "hold_timeout_s": 6.0,
}

SIX_CONTROL_REMAPS = [
    (f"/task2/joint{index}/cmd_pos", f"/task3_six/arm/joint{index}/cmd_pos")
    for index in range(1, 7)
] + [
    (f"/task2/gripper/{side}{index}_cmd_pos", f"/task3_six/gripper/{side}{index}_cmd_pos")
    for side in ("left", "right")
    for index in (3, 2, 1)
]

SERVER_REMAPS = [
    ("/task3/sort_object", "/task3_six/sort_object"),
    ("/task3/gazebo/pose/info", "/task3_six/gazebo/pose/info"),
    ("/task3/arm/joint_state", "/task3_six/arm/joint_state"),
] + [
    (f"/task3/arm/joint{index}/cmd_pos", f"/task3_six/arm/joint{index}/cmd_pos")
    for index in range(1, 7)
] + [
    (f"/task3/gripper/{side}{index}_cmd_pos", f"/task3_six/gripper/{side}{index}_cmd_pos")
    for side in ("left", "right")
    for index in (3, 2, 1)
]

BRIDGE_ARGUMENTS = [
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    "/task3_six/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/world/task3_six_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
    "/task2/gazebo/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
] + [
    f"/task2/joint{index}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double"
    for index in range(1, 7)
] + [
    f"/task2/gripper/{side}{index}_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double"
    for side in ("left", "right")
    for index in (3, 2, 1)
]


def _fast_motion_config(reference: Path) -> Path:
    """Reference poses with the fast timing blocks, written to a temp file."""
    config = yaml.safe_load(reference.read_text(encoding="utf-8"))
    config["motion"]["durations_s"] = dict(FAST_DURATIONS_S)
    config["motion"]["closed_loop"] = dict(FAST_CLOSED_LOOP)
    target = Path(tempfile.mkdtemp(prefix="task3_six_fast_")) / reference.name
    target.write_text(
        yaml.safe_dump(config, sort_keys=False, default_flow_style=None, width=200),
        encoding="utf-8",
    )
    return target


def generate_launch_description():
    six_share = Path(get_package_share_directory("task3_six_sim"))
    # The robot description lives in the shared task3_sim package, the same file
    # the reference six-grid launch uses.
    task3_share = Path(get_package_share_directory("task3_sim"))
    description_share = Path(get_package_share_directory("mycobot_description"))

    world_file = six_share / "worlds" / "task3_six_world.sdf"
    scene_config = six_share / "config" / "scene_six.yaml"
    robot_urdf = task3_share / "urdf" / "mecharm_270_gazebo.urdf"
    motion_config = _fast_motion_config(six_share / "config" / "motion_points_six.yaml")

    resource_path = str(description_share.parent)
    existing = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if existing:
        resource_path += os.pathsep + existing

    spawn_request = (
        f'sdf_filename: "{robot_urdf}", '
        'name: "mecharm_270_six", '
        'pose: {position: {x: 0.0, y: 0.0, z: 0.40}}'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Ignition Gazebo server (no GUI).",
            ),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_path),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-r", str(world_file)],
                condition=UnlessCondition(LaunchConfiguration("headless")),
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-s", "-r", str(world_file)],
                condition=IfCondition(LaunchConfiguration("headless")),
                output="screen",
            ),
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                name="task3_six_bridge",
                arguments=BRIDGE_ARGUMENTS,
                remappings=SIX_CONTROL_REMAPS
                + [("/world/task3_six_world/pose/info", "/task3_six/gazebo/pose/info")]
                + [("/task2/gazebo/joint_state", "/task3_six/arm/joint_state")],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="task3_six_robot_state_publisher",
                parameters=[{
                    "robot_description": robot_urdf.read_text(encoding="utf-8"),
                    "use_sim_time": True,
                }],
                output="screen",
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ign", "service", "-s", "/world/task3_six_world/create",
                            "--reqtype", "ignition.msgs.EntityFactory",
                            "--reptype", "ignition.msgs.Boolean", "--timeout", "5000",
                            "--req", spawn_request,
                        ],
                        output="screen",
                    ),
                    Node(
                        package="task3_six_fast",
                        executable="six_pick_sort_fast_server",
                        name="task3_six_pick_sort_server",
                        parameters=[{
                            "motion_config": str(motion_config),
                            "scene_config": str(scene_config),
                            "log_path": str(Path.home() / ".ros" / "task3_six_fast_sort_log.jsonl"),
                            "action_name": "/task3_six/sort_object",
                            # The trim itself is gain-scheduled by the node; these
                            # are the reference values it falls back to.
                            "joint_trim_gain": 2.0,
                            "joint_trim_limit_deg": 40.0,
                            "joint_error_tolerance_deg": 0.5,
                            "cartesian_follow_gain": 0.0,
                            "cartesian_follow_limit_deg": 18.0,
                            "robot_base_z": 0.40,
                            "use_sim_time": True,
                        }],
                        remappings=SERVER_REMAPS,
                        output="screen",
                    ),
                ],
            ),
        ]
    )
