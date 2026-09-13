import math

from task2_sim.real_goal_monitor import (
    JointGoalMonitor,
    reset_ready,
    within_tolerance,
)


def make_monitor(state='A_SAFE', tolerance_deg=2.0, started_at=0.0):
    return JointGoalMonitor(
        state_name=state,
        target=[0.0] * 6,
        tolerance_rad=math.radians(tolerance_deg),
        stable_required_s=0.5,
        timeout_s=20.0,
        feedback_max_age_s=0.75,
        started_at=started_at,
    )


def test_legacy_duration_does_not_finish_real_motion():
    monitor = make_monitor()
    result = monitor.evaluate(3.0, [math.radians(8.0)] * 6, 3.0)
    assert result.status == JointGoalMonitor.WAIT


def test_in_tolerance_but_not_stable_does_not_finish():
    monitor = make_monitor()
    monitor.evaluate(1.0, [math.radians(1.0)] * 6, 1.0)
    result = monitor.evaluate(1.2, [math.radians(1.0)] * 6, 1.2)
    assert result.status == JointGoalMonitor.WAIT
    assert result.stable_s < 0.5


def test_stable_goal_reaches_only_after_required_time():
    monitor = make_monitor()
    monitor.evaluate(1.0, [math.radians(1.0)] * 6, 1.0)
    result = monitor.evaluate(1.6, [math.radians(1.0)] * 6, 1.6)
    assert result.status == JointGoalMonitor.REACHED


def test_return_home_not_reached_while_error_large():
    monitor = make_monitor(state='RETURN_HOME', tolerance_deg=1.5)
    result = monitor.evaluate(5.0, [math.radians(3.0)] * 6, 5.0)
    assert result.status == JointGoalMonitor.WAIT


def test_return_home_reaches_after_stable_feedback():
    monitor = make_monitor(state='RETURN_HOME', tolerance_deg=1.5)
    monitor.evaluate(2.0, [math.radians(1.0)] * 6, 2.0)
    result = monitor.evaluate(2.6, [math.radians(1.0)] * 6, 2.6)
    assert result.status == JointGoalMonitor.REACHED


def test_stale_feedback_never_counts_as_reached():
    monitor = make_monitor()
    result = monitor.evaluate(2.0, [0.0] * 6, 1.0)
    assert result.status == JointGoalMonitor.FEEDBACK_STALE


def test_command_fallback_equivalent_missing_measured_feedback_fails():
    monitor = make_monitor()
    result = monitor.evaluate(1.0, None, None)
    assert result.status == JointGoalMonitor.FEEDBACK_STALE


def test_motion_timeout_returns_timeout_instead_of_reached():
    monitor = make_monitor()
    result = monitor.evaluate(21.0, [math.radians(5.0)] * 6, 21.0)
    assert result.status == JointGoalMonitor.TIMEOUT


def test_already_home_can_be_detected_without_resending():
    home = [math.radians(v) for v in [0.0, 0.2154, -20.4247, 0.0, 110.2092, 0.0]]
    measured = [value + math.radians(0.5) for value in home]
    assert within_tolerance(measured, home, math.radians(1.5))


def test_reset_requires_home_and_operator():
    assert not reset_ready(False, False)
    assert not reset_ready(True, False)
    assert not reset_ready(False, True)
    assert reset_ready(True, True)


def test_five_trial_boundaries_share_same_home_tolerance():
    home = [math.radians(v) for v in [0.0, 0.2154, -20.4247, 0.0, 110.2092, 0.0]]
    for offset_deg in [0.2, -0.4, 0.8, -1.0, 1.2]:
        measured = [value + math.radians(offset_deg) for value in home]
        assert within_tolerance(measured, home, math.radians(1.5))
