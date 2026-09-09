import math
from dataclasses import dataclass


@dataclass
class GoalEvaluation:
    status: str
    errors_rad: tuple
    max_error_rad: float | None
    feedback_age_s: float | None
    stable_s: float


class JointGoalMonitor:
    """Pure-Python feedback gate shared by real Task2 nodes."""

    WAIT = 'WAIT'
    WAIT_FEEDBACK = 'WAIT_FEEDBACK'
    REACHED = 'REACHED'
    FEEDBACK_STALE = 'FEEDBACK_STALE'
    TIMEOUT = 'TIMEOUT'

    def __init__(
        self,
        state_name,
        target,
        tolerance_rad,
        stable_required_s,
        timeout_s,
        feedback_max_age_s,
        started_at,
    ):
        self.state_name = str(state_name)
        self.target = tuple(float(value) for value in target)
        if len(self.target) != 6:
            raise ValueError('joint target must contain six values')

        self.tolerance_rad = float(tolerance_rad)
        self.stable_required_s = max(0.0, float(stable_required_s))
        self.timeout_s = max(0.01, float(timeout_s))
        self.feedback_max_age_s = max(0.01, float(feedback_max_age_s))
        self.started_at = float(started_at)
        self.deadline = self.started_at + self.timeout_s
        self.stable_since = None

    def evaluate(self, now, measured, measured_at):
        now = float(now)

        if measured is None or measured_at is None:
            self.stable_since = None
            waited = max(0.0, now - self.started_at)
            status = (
                self.FEEDBACK_STALE
                if waited > self.feedback_max_age_s
                else self.WAIT_FEEDBACK
            )
            return GoalEvaluation(
                status=status,
                errors_rad=tuple(),
                max_error_rad=None,
                feedback_age_s=None,
                stable_s=0.0,
            )

        measured = tuple(float(value) for value in measured)
        if len(measured) != 6:
            self.stable_since = None
            return GoalEvaluation(
                status=self.FEEDBACK_STALE,
                errors_rad=tuple(),
                max_error_rad=None,
                feedback_age_s=None,
                stable_s=0.0,
            )

        feedback_age = max(0.0, now - float(measured_at))
        if feedback_age > self.feedback_max_age_s:
            self.stable_since = None
            return GoalEvaluation(
                status=self.FEEDBACK_STALE,
                errors_rad=tuple(),
                max_error_rad=None,
                feedback_age_s=feedback_age,
                stable_s=0.0,
            )

        errors = tuple(
            abs(current - target)
            for current, target in zip(measured, self.target)
        )
        max_error = max(errors)

        if max_error <= self.tolerance_rad:
            if self.stable_since is None:
                self.stable_since = now
            stable_s = max(0.0, now - self.stable_since)
            if stable_s >= self.stable_required_s:
                return GoalEvaluation(
                    status=self.REACHED,
                    errors_rad=errors,
                    max_error_rad=max_error,
                    feedback_age_s=feedback_age,
                    stable_s=stable_s,
                )
        else:
            self.stable_since = None
            stable_s = 0.0

        if now > self.deadline:
            return GoalEvaluation(
                status=self.TIMEOUT,
                errors_rad=errors,
                max_error_rad=max_error,
                feedback_age_s=feedback_age,
                stable_s=stable_s,
            )

        return GoalEvaluation(
            status=self.WAIT,
            errors_rad=errors,
            max_error_rad=max_error,
            feedback_age_s=feedback_age,
            stable_s=stable_s,
        )


def within_tolerance(measured, target, tolerance_rad):
    if measured is None:
        return False
    measured = tuple(float(value) for value in measured)
    target = tuple(float(value) for value in target)
    if len(measured) != 6 or len(target) != 6:
        return False
    return max(abs(a - b) for a, b in zip(measured, target)) <= float(tolerance_rad)


def reset_ready(robot_home_confirmed, operator_ready):
    return bool(robot_home_confirmed and operator_ready)


def radians_to_degrees(values):
    return [math.degrees(float(value)) for value in values]
