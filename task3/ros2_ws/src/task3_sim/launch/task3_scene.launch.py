import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    task3_share = Path(get_package_share_directory("task3_sim"))
    description_share = Path(get_package_share_directory("mycobot_description"))
    world_file = task3_share / "worlds" / "task3_world.sdf"
    robot_urdf = task3_share / "urdf" / "mecharm_270_gazebo.urdf"

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
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/task3/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
        ],
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
            TimerAction(period=3.0, actions=[spawn_robot]),
        ]
    )
