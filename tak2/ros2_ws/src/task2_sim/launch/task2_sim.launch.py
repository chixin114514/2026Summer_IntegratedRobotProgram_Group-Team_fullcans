import os
from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)

from launch_ros.actions import Node


def generate_launch_description():

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    task2_share = Path(
        get_package_share_directory(
            'task2_sim'
        )
    )

    description_share = Path(
        get_package_share_directory(
            'mycobot_description'
        )
    )

    # ---------------------------------------------------------
    # Task 2 world
    # ---------------------------------------------------------

    world_file = (
        task2_share
        / 'worlds'
        / 'task2_world.sdf'
    )

    # ---------------------------------------------------------
    # Official Elephant Robotics MechArm 270 model
    # ---------------------------------------------------------

    robot_urdf = (
        description_share
        / 'urdf'
        / 'mecharm_270_m5'
        / 'mecharm_270_m5.urdf'
    )

    robot_description = (
        robot_urdf.read_text()
    )

    # ---------------------------------------------------------
    # Let Gazebo resolve package://mycobot_description/...
    # mesh paths.
    # ---------------------------------------------------------

    resource_path = str(
        description_share.parent
    )

    existing_resource_path = os.environ.get(
        'IGN_GAZEBO_RESOURCE_PATH',
        '',
    )

    if existing_resource_path:
        resource_path = (
            resource_path
            + ':'
            + existing_resource_path
        )

    set_gazebo_resource_path = (
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=resource_path,
        )
    )

    # ---------------------------------------------------------
    # Start Gazebo server
    # ---------------------------------------------------------

    gazebo_server = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-s',
            '-r',
            str(world_file),
        ],
        output='screen',
    )

    # ---------------------------------------------------------
    # Bridge Gazebo simulation time to ROS 2
    # ---------------------------------------------------------

    clock_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',

        arguments=[
            '/clock'
            '@rosgraph_msgs/msg/Clock'
            '[ignition.msgs.Clock'
        ],

        output='screen',
    )

    # ---------------------------------------------------------
    # Publish the URDF and TF tree
    # ---------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',

        name='robot_state_publisher',

        parameters=[
            {
                'robot_description':
                    robot_description,

                'use_sim_time':
                    True,
            }
        ],

        output='screen',
    )

    # ---------------------------------------------------------
    # Spawn the official MechArm 270 URDF directly through
    # Gazebo's EntityFactory service.
    #
    # This avoids ros_gz_sim / ros_ign_gazebo, whose Humble
    # binary is not available for this Jetson ARM64 target.
    # ---------------------------------------------------------

    spawn_request = (
        'sdf_filename: "'
        + str(robot_urdf)
        + '", '
        + 'name: "mecharm_270", '
        + 'pose: {'
        + 'position: {'
        + 'x: 0.0, '
        + 'y: 0.0, '
        + 'z: 0.40'
        + '}'
        + '}'
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            'ign',
            'service',

            '-s',
            '/world/task2_world/create',

            '--reqtype',
            'ignition.msgs.EntityFactory',

            '--reptype',
            'ignition.msgs.Boolean',

            '--timeout',
            '5000',

            '--req',
            spawn_request,
        ],

        output='screen',
    )

    # Gazebo needs a short time to load the world before the
    # entity creation service is called.
    delayed_robot_spawn = TimerAction(
        period=3.0,

        actions=[
            spawn_robot,
        ],
    )

    # ---------------------------------------------------------
    # Current complete Task 2 simulation launch
    # ---------------------------------------------------------

    return LaunchDescription([
        set_gazebo_resource_path,
        gazebo_server,
        clock_bridge,
        robot_state_publisher,
        delayed_robot_spawn,
    ])
