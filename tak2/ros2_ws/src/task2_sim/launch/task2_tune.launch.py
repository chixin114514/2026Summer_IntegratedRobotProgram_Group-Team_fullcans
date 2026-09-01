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

    world_file = (
        task2_share
        / 'worlds'
        / 'task2_world.sdf'
    )

    robot_file = (
        task2_share
        / 'urdf'
        / 'mecharm_270_gazebo.urdf'
    )

    robot_description = (
        robot_file.read_text()
    )

    resource_path = str(
        description_share.parent
    )

    old_resource_path = (
        os.environ.get(
            'IGN_GAZEBO_RESOURCE_PATH',
            '',
        )
    )

    if old_resource_path:

        resource_path += (
            ':'
            +
            old_resource_path
        )

    gazebo = ExecuteProcess(
        cmd=[
            'ign',
            'gazebo',
            '-r',
            str(world_file),
        ],
        output='screen',
    )

    bridge_arguments = [

        (
            '/clock'
            '@rosgraph_msgs/msg/Clock'
            '[ignition.msgs.Clock'
        ),

    ]

    for index in range(
        1,
        7,
    ):

        bridge_arguments.append(

            (
                f'/task2/joint'
                f'{index}/cmd_pos'
                '@std_msgs/msg/Float64'
                ']ignition.msgs.Double'
            )

        )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=bridge_arguments,
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable=(
            'robot_state_publisher'
        ),
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

    spawn_request = (
        'sdf_filename: "'
        + str(robot_file)
        + '", '
        + 'name: "mecharm_270", '
        + 'pose: {position: {'
        + 'x: 0.0, '
        + 'y: 0.0, '
        + 'z: 0.40'
        + '}}'
    )

    spawn = ExecuteProcess(
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

    tuner_file = (
        task2_share
        / 'scripts'
        / 'joint_tuner.py'
    )

    tuner = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            str(tuner_file),
        ],
        output='screen',
    )

    open_panel = ExecuteProcess(
        cmd=[
            'xdg-open',
            'http://127.0.0.1:8080',
        ],
        output='screen',
    )

    return LaunchDescription([

        SetEnvironmentVariable(
            name=(
                'IGN_GAZEBO_RESOURCE_PATH'
            ),
            value=resource_path,
        ),

        gazebo,

        bridge,

        robot_state_publisher,

        TimerAction(
            period=3.0,
            actions=[
                spawn
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                tuner
            ],
        ),

        TimerAction(
            period=7.0,
            actions=[
                open_panel
            ],
        ),

    ])
