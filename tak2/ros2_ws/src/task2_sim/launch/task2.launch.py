import os

from pathlib import Path

import yaml

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


def read_yaml(path):

    with Path(path).open(
        'r',
        encoding='utf-8',
    ) as file:

        return yaml.safe_load(
            file
        )


def script_process(
    task2_share,
    script_name,
    use_sim_time,
):

    command = [

        '/usr/bin/python3',

        str(
            task2_share
            /
            'scripts'
            /
            script_name
        ),
    ]

    if use_sim_time:

        command.extend([
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ])

    return ExecuteProcess(
        cmd=command,
        output='screen',
    )


def generate_launch_description():

    # ========================================================
    # Package paths
    # ========================================================

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

    config_dir = (
        task2_share
        /
        'config'
    )

    device = read_yaml(
        config_dir
        /
        'device.yaml'
    )

    communication = read_yaml(
        config_dir
        /
        'communication.yaml'
    )

    mode = int(
        device[
            'mode'
        ]
    )

    if mode not in (
        0,
        1,
    ):

        raise RuntimeError(
            'device.yaml mode must be '
            '0 (simulation) or 1 (real robot).'
        )

    simulation_mode = (
        mode == 0
    )

    print(
        '=========================================='
    )

    print(
        'TASK 2 SYSTEM MODE: '
        +
        (
            'SIMULATION'
            if simulation_mode
            else
            'REAL ROBOT'
        )
    )

    print(
        '=========================================='
    )

    # ========================================================
    # Robot description
    # ========================================================

    robot_urdf = (
        task2_share
        /
        'urdf'
        /
        'mecharm_270_gazebo.urdf'
    )

    robot_description = (
        robot_urdf.read_text()
    )

    robot_state_publisher = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        name='robot_state_publisher',

        parameters=[{

            'robot_description':
                robot_description,

            'use_sim_time':
                simulation_mode,
        }],

        remappings=[

            (
                '/joint_states',
                '/task2/robot_state',
            ),
        ],

        output='screen',
    )

    actions = [
        robot_state_publisher,
    ]

    # ========================================================
    # MODE 0: Gazebo / Ignition
    # ========================================================

    if simulation_mode:

        world_file = (
            task2_share
            /
            'worlds'
            /
            'task2_world.sdf'
        )

        resource_path = str(
            description_share.parent
        )

        existing_resource_path = (
            os.environ.get(
                'IGN_GAZEBO_RESOURCE_PATH',
                '',
            )
        )

        if existing_resource_path:

            resource_path += (
                ':'
                +
                existing_resource_path
            )

        actions.append(

            SetEnvironmentVariable(
                name='IGN_GAZEBO_RESOURCE_PATH',
                value=resource_path,
            )
        )

        # ----------------------------------------------------
        # Gazebo GUI + server
        # ----------------------------------------------------

        gazebo = ExecuteProcess(

            cmd=[

                'ign',
                'gazebo',
                '-r',
                str(
                    world_file
                ),
            ],

            output='screen',
        )

        actions.append(
            gazebo
        )

        # ----------------------------------------------------
        # ROS <-> Ignition bridge
        # ----------------------------------------------------

        bridge_arguments = [

            (
                '/clock'
                '@rosgraph_msgs/msg/Clock'
                '[ignition.msgs.Clock'
            ),

            (
                '/task2/gazebo/joint_state'
                '@sensor_msgs/msg/JointState'
                '[ignition.msgs.Model'
            ),

            (
                '/world/task2_world/pose/info'
                '@tf2_msgs/msg/TFMessage'
                '[ignition.msgs.Pose_V'
            ),
        ]

        for index in range(
            1,
            7,
        ):

            bridge_arguments.append(

                (
                    f'/task2/joint{index}/cmd_pos'
                    '@std_msgs/msg/Float64'
                    ']ignition.msgs.Double'
                )
            )

        bridge_arguments.extend([

            (
                '/task2/gripper/left_cmd_pos'
                '@std_msgs/msg/Float64'
                ']ignition.msgs.Double'
            ),

            (
                '/task2/gripper/right_cmd_pos'
                '@std_msgs/msg/Float64'
                ']ignition.msgs.Double'
            ),
        ])

        bridge = Node(

            package='ros_ign_bridge',

            executable='parameter_bridge',

            arguments=
                bridge_arguments,

            output='screen',
        )

        actions.append(
            bridge
        )

        # ----------------------------------------------------
        # Spawn MechArm model
        # ----------------------------------------------------

        spawn_request = (

            'sdf_filename: "'
            +
            str(
                robot_urdf
            )
            +
            '", '

            'name: "mecharm_270", '

            'pose: {'

                'position: {'

                    'x: 0.0, '
                    'y: 0.0, '
                    'z: 0.40'

                '}'

            '}'
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

        actions.append(

            TimerAction(
                period=3.0,
                actions=[
                    spawn_robot
                ],
            )
        )

    # ========================================================
    # MODE 1: physical MechArm 270
    # ========================================================

    else:

        real_driver = (
            script_process(
                task2_share,
                'real_robot_driver.py',
                False,
            )
        )

        actions.append(
            real_driver
        )

    # ========================================================
    # Device-independent core nodes
    # ========================================================

    core_nodes = [

        script_process(
            task2_share,
            'arm_interface.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'gripper_interface.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'safety_monitor.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'state_publisher.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'task_manager.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'result_monitor.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'trial_reset_interface.py',
            simulation_mode,
        ),

        script_process(
            task2_share,
            'task_logger.py',
            simulation_mode,
        ),
    ]

    # In simulation allow Gazebo to exist and the robot
    # to be spawned before starting controllers.

    core_delay = (
        5.0
        if simulation_mode
        else
        2.0
    )

    actions.append(

        TimerAction(
            period=core_delay,
            actions=core_nodes,
        )
    )

    # ========================================================
    # Experiment manager starts LAST.
    #
    # It automatically runs the five acceptance trials.
    # ========================================================

    experiment_manager = (
        script_process(
            task2_share,
            'experiment_manager.py',
            simulation_mode,
        )
    )

    experiment_delay = (
        8.0
        if simulation_mode
        else
        4.0
    )

    actions.append(

        TimerAction(
            period=experiment_delay,
            actions=[
                experiment_manager
            ],
        )
    )

    return LaunchDescription(
        actions
    )
