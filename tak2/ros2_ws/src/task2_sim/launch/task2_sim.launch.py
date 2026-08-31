from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    ExecuteProcess,
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
    # Gazebo world
    # ---------------------------------------------------------

    world_file = (
        task2_share
        / 'worlds'
        / 'task2_world.sdf'
    )

    # ---------------------------------------------------------
    # Official MechArm 270 model
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
    # Start Ignition Gazebo
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
    # Bridge Gazebo simulation time into ROS 2
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
    # Publish robot description and robot TF tree
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
    # Spawn MechArm 270 into Gazebo
    #
    # Table top:
    #   z = 0.40 m
    #
    # Therefore the robot base is mounted at z = 0.40 m.
    # ---------------------------------------------------------

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',

        arguments=[
            '-name',
            'mecharm_270',

            '-topic',
            '/robot_description',

            '-x',
            '0.0',

            '-y',
            '0.0',

            '-z',
            '0.40',

            '-Y',
            '0.0',
        ],

        output='screen',
    )

    # Give Gazebo time to create the world before spawning robot.
    delayed_robot_spawn = TimerAction(
        period=3.0,

        actions=[
            spawn_robot,
        ],
    )

    # ---------------------------------------------------------
    # One launch file starts the current simulation stack
    # ---------------------------------------------------------

    return LaunchDescription([
        gazebo_server,
        clock_bridge,
        robot_state_publisher,
        delayed_robot_spawn,
    ])
