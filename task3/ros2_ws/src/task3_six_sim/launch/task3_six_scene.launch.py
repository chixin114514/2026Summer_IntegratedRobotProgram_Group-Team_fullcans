import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def generate_launch_description():
    six_share = Path(get_package_share_directory("task3_six_sim"))
    task3_share = Path(get_package_share_directory("task3_sim"))
    description_share = Path(get_package_share_directory("mycobot_description"))
    world_file = six_share / "worlds" / "task3_six_world.sdf"
    # Reuse the field-proven robot description unchanged.  Its arm joint loops
    # are soft, so the six-grid server (below) adds a software integral trim on
    # the arm command to make the *measured* joints reach the commanded pose.
    robot_urdf = task3_share / "urdf" / "mecharm_270_gazebo.urdf"
    motion_config = six_share / "config" / "motion_points_six.yaml"
    scene_config = six_share / "config" / "scene_six.yaml"

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
                parameters=[{"robot_description": robot_urdf.read_text(encoding="utf-8"), "use_sim_time": True}],
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
                        package="task3_six_sim",
                        executable="six_pick_sort_server",
                        name="task3_six_pick_sort_server",
                        parameters=[{
                            "motion_config": str(motion_config),
                            "scene_config": str(scene_config),
                            "log_path": str(Path.home() / ".ros" / "task3_six_sort_log.jsonl"),
                            "action_name": "/task3_six/sort_object",
                            "joint_trim_gain": 2.0,
                            "joint_trim_limit_deg": 40.0,
                            "joint_error_tolerance_deg": 0.2,
                            # The Cartesian follow loop is deliberately off: with
                            # only the joint trim live every step's target is
                            # static, so the trim actually converges (measured
                            # joint error < 0.25 deg) and the tool lands on the
                            # FK prediction, which is exactly the block centre.
                            # An earlier running integral follow had a moving
                            # target, so the trim chased it and never settled.
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
