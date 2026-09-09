import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


# These are the immutable Gazebo transport endpoints from the imported URDF.
# The remapping table below is the only Task3-owned adapter to those names.
CONTROL_REMAPS = [
    ("/task2/joint1/cmd_pos", "/task3/arm/joint1/cmd_pos"),
    ("/task2/joint2/cmd_pos", "/task3/arm/joint2/cmd_pos"),
    ("/task2/joint3/cmd_pos", "/task3/arm/joint3/cmd_pos"),
    ("/task2/joint4/cmd_pos", "/task3/arm/joint4/cmd_pos"),
    ("/task2/joint5/cmd_pos", "/task3/arm/joint5/cmd_pos"),
    ("/task2/joint6/cmd_pos", "/task3/arm/joint6/cmd_pos"),
    ("/task2/gripper/left3_cmd_pos", "/task3/gripper/left3_cmd_pos"),
    ("/task2/gripper/left2_cmd_pos", "/task3/gripper/left2_cmd_pos"),
    ("/task2/gripper/left1_cmd_pos", "/task3/gripper/left1_cmd_pos"),
    ("/task2/gripper/right3_cmd_pos", "/task3/gripper/right3_cmd_pos"),
    ("/task2/gripper/right2_cmd_pos", "/task3/gripper/right2_cmd_pos"),
    ("/task2/gripper/right1_cmd_pos", "/task3/gripper/right1_cmd_pos"),
]

BRIDGE_ARGUMENTS = [
    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    "/task3/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
    "/world/task3_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
    "/task2/joint1/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/joint2/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/joint3/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/joint4/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/joint5/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/joint6/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/gripper/left3_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/gripper/left2_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/gripper/left1_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/gripper/right3_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/gripper/right2_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
    "/task2/gripper/right1_cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double",
]


def generate_launch_description():
    task3_share = Path(get_package_share_directory("task3_sim"))
    description_share = Path(get_package_share_directory("mycobot_description"))
    world_file = task3_share / "worlds" / "task3_world.sdf"
    robot_urdf = task3_share / "urdf" / "mecharm_270_gazebo.urdf"
    motion_config = task3_share / "config" / "motion_points.yaml"
    scene_config = task3_share / "config" / "scene.yaml"

    resource_path = str(description_share.parent)
    existing_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if existing_resource_path:
        resource_path += os.pathsep + existing_resource_path

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_urdf.read_text(encoding="utf-8"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="task3_bridge",
        arguments=BRIDGE_ARGUMENTS,
        remappings=CONTROL_REMAPS
        + [("/world/task3_world/pose/info", "/task3/gazebo/pose/info")],
        output="screen",
    )

    spawn_request = (
        f'sdf_filename: "{robot_urdf}", '
        'name: "mecharm_270", '
        'pose: {position: {x: 0.0, y: 0.0, z: 0.40}}'
    )
    spawn_robot = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            "/world/task3_world/create",
            "--reqtype",
            "ignition.msgs.EntityFactory",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            spawn_request,
        ],
        output="screen",
    )

    pick_sort_server = Node(
        package="task3_sim",
        executable="pick_sort_server",
        name="pick_sort_server",
        parameters=[
            {
                "motion_config": str(motion_config),
                "scene_config": str(scene_config),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=resource_path,
            ),
            ExecuteProcess(
                cmd=["ign", "gazebo", "-r", str(world_file)],
                output="screen",
            ),
            bridge,
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn_robot, pick_sort_server]),
        ]
    )
