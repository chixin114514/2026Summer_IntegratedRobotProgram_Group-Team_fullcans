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
