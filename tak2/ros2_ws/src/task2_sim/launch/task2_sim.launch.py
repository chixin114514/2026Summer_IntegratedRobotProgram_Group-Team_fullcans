from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import ExecuteProcess


def generate_launch_description():

    package_share = Path(
        get_package_share_directory(
            'task2_sim'
        )
    )

    world_file = (
        package_share
        / 'worlds'
        / 'task2_world.sdf'
    )

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

    return LaunchDescription([
        gazebo_server,
    ])
