import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = (
    Path(__file__)
    .resolve()
    .parents[1]
)

sys.path.insert(
    0,
    str(PACKAGE_ROOT),
)

from task2_sim.kinematics import MechArmKinematics
from task2_sim.runtime_config import Task2Config


def load_config():
    return Task2Config(
        PACKAGE_ROOT / 'config'
    )


def test_all_formal_waypoints_point_tool_vertically_down():

    config = load_config()

    kin = MechArmKinematics(
        config
    )

    poses_deg = {
        'HOME':
            config.task['home']['joints_deg'],
    }

    poses_deg.update(
        {
            name.upper():
                values
            for name, values
            in config.task[
                'kinematics'
            ][
                'upright_seed_deg'
            ].items()
        }
    )

    for name, values in poses_deg.items():

        joints = [
            math.radians(
                float(value)
            )
            for value in values
        ]

        rotation = (
            kin.forward_rotation(
                joints
            )
        )

        # Tool local +Z compared with world -Z.
        dot = max(
            -1.0,
            min(
                1.0,
                -rotation[2][2],
            ),
        )

        tilt_deg = math.degrees(
            math.acos(
                dot
            )
        )

        assert tilt_deg < 0.05, (
            name,
            tilt_deg,
            values,
        )


def test_gazebo_initial_arm_pose_matches_formal_home():

    config = load_config()

    home = [
        math.radians(
            float(value)
        )
        for value in
        config.task[
            'home'
        ][
            'joints_deg'
        ]
    ]

    tree = ET.parse(
        PACKAGE_ROOT
        /
        'urdf'
        /
        'mecharm_270_gazebo.urdf'
    )

    root = tree.getroot()

    joint_names = [
        'joint1_to_base',
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
    ]

    initial = {}

    for plugin in root.findall(
        './/plugin'
    ):

        joint_name = plugin.findtext(
            'joint_name'
        )

        value = plugin.findtext(
            'initial_position'
        )

        if (
            joint_name in joint_names
            and
            value is not None
        ):

            initial[
                joint_name
            ] = float(value)

    assert set(initial) == set(
        joint_names
    )

    for index, name in enumerate(
        joint_names
    ):

        assert abs(
            initial[name]
            -
            home[index]
        ) < 1e-6


def test_simulation_uses_measured_convergence():

    text = (
        PACKAGE_ROOT
        /
        'task2_sim'
        /
        'task_manager.py'
    ).read_text(
        encoding='utf-8'
    )

    assert (
        'SIM_TRIAL_RESYNC'
        in text
    )

    assert (
        'SIM_GOAL_WAIT'
        in text
    )

    assert (
        'SIM_GOAL_REACHED'
        in text
    )

    assert (
        'SIM_GOAL_TIMEOUT'
        in text
    )

    # Do not depend on exact source indentation/formatting.
    # The contract is that simulation start_motion uses the
    # measured Gazebo joint state as its physical start pose.
    assert (
        'start_pose = list('
        in text
    )

    assert (
        'self.current_joint_state'
        in text
    )

    assert (
        'SIMULATION PHYSICAL RESYNC'
        in text
    )
