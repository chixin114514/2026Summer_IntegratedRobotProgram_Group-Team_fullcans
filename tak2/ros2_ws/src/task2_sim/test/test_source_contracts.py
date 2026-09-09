from pathlib import Path


ROOT = Path(__file__).resolve().parents[1] / 'task2_sim'


def text(name):
    return (ROOT / name).read_text(encoding='utf-8')


def test_return_home_confirmation_gates_completed():
    source = text('task_manager.py')
    assert 'COMPLETION_BLOCKED: RETURN_HOME has not been confirmed' in source
    assert 'RETURN_HOME_CONFIRMED' in source


def test_driver_communication_fault_stops_before_fault_publish():
    source = text('real_robot_driver.py')
    block = source.split('def communication_fault', 1)[1].split('def arm_command_callback', 1)[0]
    assert 'self.perform_stop()' in block
    assert 'self.publish_fault(reason)' in block
    assert block.index('self.perform_stop()') < block.index('self.publish_fault(reason)')


def test_safety_reset_is_subscribed_by_all_latched_motion_nodes():
    task_manager = text('task_manager.py')
    driver = text('real_robot_driver.py')
    safety = text('safety_monitor.py')
    for source in (task_manager, driver, safety):
        assert "'safety_reset_topic'" in source
    assert 'def safety_reset_callback' in task_manager
    assert 'def safety_reset_callback' in driver
    assert 'def reset_callback' in safety


def test_real_reset_does_not_own_arm_home_motion():
    source = text('trial_reset_interface.py')

    assert 'requested_arm_command_topic' not in source
    assert 'REAL_GOAL_SENT state=RESET_HOME' not in source

    assert (
        'HOME motion is owned exclusively'
        in source
    )


def test_real_driver_sends_without_blocking_preflight():
    source = text('real_robot_driver.py')

    assert 'def prepare_hardware_for_motion' not in source

    # The old synchronous diagnostic that returned -1 must not
    # run in the arm command path.
    assert "'get_error_information'" not in source

    assert 'REAL_ARM_TX ' in source
    assert 'self.robot.send_angles' in source

    block = (
        source
        .split(
            'def arm_command_callback',
            1,
        )[1]
        .split(
            'def gripper_command_callback',
            1,
        )[0]
    )

    assert (
        'prepare_hardware_for_motion'
        not in block
    )

    assert (
        'self.robot.send_angles'
        in block
    )
