"""Six-grid pick-and-place server tuned for shorter runs.

Same scene, same world, same arm, same grasp geometry as ``task3_six_sim``.  The
only differences are how long a step waits before it is allowed to move on, and
how the arm command is compensated.  Nothing in ``task3_six_sim`` or ``task3_sim``
is modified or copied; this module subclasses the six-grid server and overrides
two methods.

Where the time used to go
-------------------------
A step ends as soon as the measured joints track the commanded pose, otherwise it
waits out ``hold_timeout_s``.  Measured on a full run, almost every one of the 21
steps per block used the whole timeout, for 288 s per block.  Two separate reasons
made the exit condition unreachable:

* The tolerance was one number for every step (0.2 deg).  Steps that only travel
  between poses plateau at 1-2 deg, because the arm joints run very soft position
  loops and the folded pose sits within a few degrees of the J5 limit, where the
  compensation is clamped and the error simply cannot be driven further down.
  Those steps were waiting 12 s for an error that was never going to shrink.
* The settle velocity was 0.03 rad/s for every step too, so a step that had
  already arrived was still held back while the last fraction of a degree crept.

What this module changes
------------------------
Steps whose job is to put the jaws around a block, or to set a block down, keep
the tight tolerance and the tight settle velocity.  Every transit step gets a
loose pair instead, because a couple of degrees at the folded or transfer pose
moves nothing that matters.

The trim gain is scheduled rather than raised.  A flat increase is not available
here: measured against Ignition, a gain of 6.0 pushes the loop into a limit cycle
and the residual error *grows* from 0.14 deg to 1-5 deg.  The gain is therefore
3.0 while the error is above 2 deg, where the loop has plenty of margin and the
extra speed is free, and 2.0 below that, which is the value the reference
configuration uses.  A deadband stops the last hundredth of a degree from being
chased with the noise that comes with it.
"""

from __future__ import annotations

import math
import time

from task3_sim.pick_sort_server import MAX_JOINT_AGE_S
from task3_six_sim.six_pick_sort_server import SixPickSortServer


# Steps that are given the tight budget.  The three descent steps slide the open
# jaws past a block, and the two placement steps are where the closed loop that
# lands the block does its work, so all five need the tool where it matters and
# the loop given time to finish.  Every other step is transit: the folded pose,
# the swing between poses, carrying the block, lifting away, retracting.  A
# couple of degrees there moves nothing that matters, and the next step keeps
# correcting anyway, so waiting for it is pure dead time.  In particular the
# press-into-the-block steps (PICK, CLOSE) can never satisfy a tight joint
# tolerance at all -- the block is in the way -- so they used to burn the full
# timeout on every block.
TIGHT_STAGES = frozenset((
    "PICK_DESCEND_1",
    "PICK_DESCEND_2",
    "PICK_DESCEND_3",
    "BIN_PLACE",
    "RELEASE",
))
TIGHT_TOLERANCE_DEG = 0.5
LOOSE_TOLERANCE_DEG = 3.0
TIGHT_SETTLE_VELOCITY_RAD_S = 0.05
LOOSE_SETTLE_VELOCITY_RAD_S = 0.15

# Scheduled trim gain.  FAST_GAIN applies while the error is comfortably larger
# than the plant's own lag can overshoot into; SLOW_GAIN is the reference value.
FAST_GAIN = 3.0
SLOW_GAIN = 2.0
GAIN_SWITCH_DEG = 2.0
DEADBAND_DEG = 0.05


class FastPickSortServer(SixPickSortServer):
    """Six-grid server with a per-step accuracy budget."""

    def _run_step(self, goal_handle, step, index: int, total: int) -> bool:
        # Set the budget for this step before delegating; the parent reads both
        # values on every tick of the hold loop.
        tight = step.stage in TIGHT_STAGES
        self._trim_tolerance = TIGHT_TOLERANCE_DEG if tight else LOOSE_TOLERANCE_DEG
        self._settle_velocity_rad_s = (
            TIGHT_SETTLE_VELOCITY_RAD_S if tight else LOOSE_SETTLE_VELOCITY_RAD_S
        )
        return super()._run_step(goal_handle, step, index, total)

    def _integrate_trim(self, target_deg, dt: float) -> float:
        """Fold the measured joint error into the command; return the worst |error|.

        Differs from the reference implementation in two ways: the gain is
        scheduled on the error magnitude, and an error inside the deadband is left
        alone instead of being chased.
        """
        trim = getattr(self, "_trim", None)
        if trim is None:
            return float("inf")
        with self._joint_lock:
            sample = self._measured_arm
        if sample is None:
            return float("inf")
        timestamp, positions, _velocities = sample
        if time.monotonic() - timestamp > MAX_JOINT_AGE_S:
            return float("inf")
        worst = 0.0
        for index, (target, measured) in enumerate(zip(target_deg, positions)):
            # step poses are degrees; the bridged JointState reports radians.
            error = float(target) - math.degrees(float(measured))
            worst = max(worst, abs(error))
            if abs(error) < DEADBAND_DEG:
                continue
            gain = FAST_GAIN if abs(error) > GAIN_SWITCH_DEG else SLOW_GAIN
            value = trim[index] + gain * error * dt
            # Anti-windup: never integrate a trim that would command the joint
            # past its limit, otherwise the arm swings somewhere unrelated.
            low = max(-self._trim_limit,
                      self._lower[index] + self._limit_margin - float(target))
            high = min(self._trim_limit,
                       self._upper[index] - self._limit_margin - float(target))
            value = 0.0 if low > high else max(low, min(high, value))
            trim[index] = value
        return worst


def main(args=None) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = FastPickSortServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
