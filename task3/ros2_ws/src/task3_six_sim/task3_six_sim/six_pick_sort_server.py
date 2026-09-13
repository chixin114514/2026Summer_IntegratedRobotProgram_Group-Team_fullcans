"""Six-grid pick-and-place Action server.

Behaviourally identical to the shared ``task3_sim`` server, with three
additions that together make the sort repeatable on this robot:

1. A software integral trim on the arm command.
   The shared robot description runs the six arm joints with very soft position
   loops (``p_gain`` 4..8).  Measured against Ignition, the joints settle 3-10
   deg short of the commanded angle and keep creeping for tens of seconds, which
   moves the tool 20-35 mm away from the intended pose -- wider than the jaw
   opening, and path dependent, so no single fixed pose fixes it.  While a step
   is held the server integrates the *measured* joint error into the command it
   publishes, so the measured joints converge onto the commanded pose.

2. An optional closed-loop Cartesian servo on the measured object position
   (``cartesian_follow_gain``, **0.0 by default = off**).
   It exists because a few millimetres of residual error makes a splayed pad
   scrape the block on the way down.  Measured against Ignition, though, an
   *integral* Cartesian loop while the reference is still ramping winds itself
   up and never settles, which drives the tool off the block far worse than the
   error it was meant to remove.  With the loop off the reference of every step
   is static, the joint trim converges to < 0.25 deg, and the tool lands on the
   forward-kinematics prediction -- which is exactly the block centre (verified
   to 0.0 mm for all six grids).  The code path is kept because it is useful for
   diagnostics and the gain is a parameter.

3. A closed-loop placement correction (``PLACE_STAGES``).
   The grasp sits ~13 mm below the jaw midpoint, slips a little in the jaws and
   the real pad midpoint sags ~10 mm below the forward-kinematics prediction at
   this reach, so a fixed slot pose drives the block into the floor: the arm
   stalls, the joint trim saturates and the block is let go wherever the jaws
   happen to be (or is dropped from a height and topples out of the region).
   The measured block-to-slot offset seeds the correction on the settled
   approach pose, and the block's own measured position is then integrated onto
   the slot while the descent and the release are held.

No file in the shared ``task3_sim`` package, its robot description, or the
legacy single-table simulation is touched.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage

from task3_sim.pick_sort_server import (
    MAX_JOINT_AGE_S,
    POSE_TOPIC,
    PickSortServer,
    _smoothstep,
)

from . import kinematics

PAD_LINKS = ("gripper_left1", "gripper_right1")
# Every stage from the approach inwards keeps the object-relative servo live: the
# jaws must already be centred when their lower corners pass the block's top, so
# the loop cannot be allowed to freeze during the descent ramps.
FOLLOW_PREFIX = "PICK"
FOLLOW_EXTRA = ("CLOSE",)
# The first approach pose is the compact folded one at the table centre, where a
# Cartesian correction would fight the fold rather than approach the block.
FOLLOW_SKIP = ("PICK_APPROACH_1",)
UPRIGHT_MIN_Z_M = 0.415      # a toppled 50 mm block drops below this
FOLLOW_MAX_RADIUS_M = 0.06   # ignore poses that are not the intended block
# Placement: stages that should land the *block*, not the jaw midpoint, on the
# slot.  The correction is measured on the settled approach pose and then only
# integrated while a step is held, so it cannot wind up against the floor.
PLACE_ENTRY_STAGE = "BIN_ABOVE"
PLACE_STAGES = (PLACE_ENTRY_STAGE, "BIN_PLACE", "RELEASE")
# Only the steps that actually reach down to the floor drive the block's height;
# pulling the block to its rest height while still 100 mm up would just drag the
# arm down through the whole approach.
PLACE_Z_STAGES = ("BIN_PLACE", "RELEASE")
# The collection regions sit on a 5 mm slab whose top is at z = 0.405, so a block
# standing inside them rests with its centre at 0.430, not the 0.425 of the bare
# table.  The target is deliberately a few millimetres above that: the residual
# demand then always points up, which the arm can satisfy, instead of pushing
# into the slab forever.
PLACE_OBJECT_Z_M = 0.4330
PLACE_MAX_SHIFT_M = 0.05
PLACE_LIMIT_DEG = 25.0
PLACE_GAIN = 1.5
# Downward demand that produces no downward progress for this long is dropped.
PLACE_Z_STALL_S = 1.0


def _quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


class SixPickSortServer(PickSortServer):
    """Pick-and-place server with closed-loop arm command compensation."""

    def __init__(self) -> None:
        super().__init__()
        # The plant only delivers roughly a third of a commanded joint change, so
        # nulling a 10-15 deg shortfall needs a large command offset.  The trim is
        # an integrator, so it converges to whatever offset is required; the clamp
        # only has to be wide enough not to bind.
        self.declare_parameter("joint_trim_gain", 2.0)
        self.declare_parameter("joint_trim_limit_deg", 40.0)
        self.declare_parameter("joint_error_tolerance_deg", 0.2)
        self.declare_parameter("cartesian_follow_gain", 0.0)
        self.declare_parameter("cartesian_follow_limit_deg", 18.0)
        self.declare_parameter("robot_base_z", 0.40)
        self._trim_gain = float(self.get_parameter("joint_trim_gain").value)
        self._trim_limit = float(self.get_parameter("joint_trim_limit_deg").value)
        self._trim_tolerance = float(self.get_parameter("joint_error_tolerance_deg").value)
        self._follow_gain = float(self.get_parameter("cartesian_follow_gain").value)
        self._follow_limit = float(self.get_parameter("cartesian_follow_limit_deg").value)
        self._base_z = float(self.get_parameter("robot_base_z").value)
        limits = self.motion_config.get("joint_limits_deg", {}) or {}
        self._lower = [float(value) for value in limits.get("lower", [-160.0] * 6)]
        self._upper = [float(value) for value in limits.get("upper", [160.0] * 6)]
        if len(self._lower) != 6 or len(self._upper) != 6:
            raise ValueError("motion configuration has invalid joint limits")
        self._limit_margin = 3.0
        self._open = float(self.motion_config["gripper"]["open_rad"])
        self._trim = [0.0] * 6
        self._follow = [0.0] * 6
        self._follow_object = ""
        self._follow_centre = None
        self._place_bin_id = ""
        self._place_target = None
        self._place_fix = [0.0] * 6
        self._place_gain = PLACE_GAIN
        self._place_z_ref = None
        self._place_z_ticks = 0

        self._link_lock = threading.Lock()
        self._link_poses = {}
        self.create_subscription(
            TFMessage,
            POSE_TOPIC,
            self._link_callback,
            10,
            callback_group=self._callback_group,
        )
        self._log(
            "arm_compensation_enabled",
            trim_gain=self._trim_gain,
            trim_limit_deg=self._trim_limit,
            trim_tolerance_deg=self._trim_tolerance,
            follow_gain=self._follow_gain,
            follow_limit_deg=self._follow_limit,
        )

    # ------------------------------------------------------------- pose input
    def _link_callback(self, message: TFMessage) -> None:
        """Keep the gripper pad frames so the real tool centre is known."""
        for transform in message.transforms:
            leaf = str(getattr(transform, "child_frame_id", "")).split("::")[-1]
            if leaf in PAD_LINKS:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                with self._link_lock:
                    self._link_poses[leaf] = (
                        float(translation.x), float(translation.y), float(translation.z),
                        float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w),
                    )

    def _measured_tool_centre(self):
        """World XYZ of the midpoint between the two rubber pads."""
        with self._link_lock:
            samples = {name: self._link_poses.get(name) for name in PAD_LINKS}
        if any(value is None for value in samples.values()):
            return None
        centres = []
        for name in PAD_LINKS:
            x, y, z, qx, qy, qz, qw = samples[name]
            rotation = _quat_to_matrix(qx, qy, qz, qw)
            local = kinematics.PAD_LOCAL[name]
            centres.append([
                (x, y, z + self._base_z)[i] + sum(rotation[i][j] * local[j] for j in range(3))
                for i in range(3)
            ])
        return [(centres[0][i] + centres[1][i]) / 2.0 for i in range(3)]

    # --------------------------------------------------------------- command
    def _publish_arm(self, arm_deg) -> None:
        values = tuple(float(value) for value in arm_deg)
        trim = getattr(self, "_trim", None)
        follow = getattr(self, "_follow", None)
        if trim is None or follow is None:
            super()._publish_arm(values)
            return
        lower = getattr(self, "_lower", None)
        upper = getattr(self, "_upper", None)
        out = []
        for index, value in enumerate(values):
            total = value + trim[index] + follow[index]
            if lower is not None:
                # A compensation that pushes a joint past its limit makes the arm
                # swing somewhere unrelated, so the published command is clamped.
                total = max(lower[index], min(upper[index], total))
            out.append(total)
        super()._publish_arm(tuple(out))

    def _integrate_trim(self, target_deg, dt: float) -> float:
        """Fold the measured joint error into the command; return the worst |error|."""
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
            value = trim[index] + self._trim_gain * error * dt
            # Anti-windup: never integrate a trim that would command the joint
            # past its limit, otherwise the arm swings somewhere unrelated.
            low = max(-self._trim_limit,
                      self._lower[index] + self._limit_margin - float(target))
            high = min(self._trim_limit,
                       self._upper[index] - self._limit_margin - float(target))
            value = 0.0 if low > high else max(low, min(high, value))
            trim[index] = value
        return worst

    def _refresh_follow(self, nominal_deg) -> None:
        """Drive the jaws onto the measured object (Cartesian integral).

        A proportional law is not enough here: the plant only delivers about a
        third of a commanded joint change, so the residual after a proportional
        step is still several millimetres -- more than the clearance between the
        splayed pads and the block.  Integrating the Cartesian error makes the
        tool actually arrive, and all three axes are closed because the jaws must
        also straddle the block at the right height.
        """
        if self._follow_gain <= 0.0 or not self._follow_object:
            return
        tool = self._measured_tool_centre()
        pose = self._current_pose(self._follow_object)
        if tool is None or pose is None or self._follow_centre is None:
            return
        object_x, object_y, object_z = pose[1], pose[2], pose[3]
        if object_z < UPRIGHT_MIN_Z_M:
            return                                    # toppled: do not chase it
        if math.hypot(object_x - self._follow_centre[0],
                      object_y - self._follow_centre[1]) > FOLLOW_MAX_RADIUS_M:
            return
        target_z = kinematics.pad_mid(nominal_deg, self._open, self._base_z)[2]
        step = kinematics.xyz_correction(
            nominal_deg,
            self._open,
            (object_x - tool[0], object_y - tool[1], target_z - tool[2]),
            gain=1.0,
            limit_deg=self._follow_limit,
            base_z=self._base_z,
        )
        for index in range(6):
            value = self._follow[index] + step[index] * self._follow_gain / self._rate_hz
            value = max(-self._follow_limit, min(self._follow_limit, value))
            # never command a joint past its limit
            value = max(self._lower[index] + self._limit_margin - nominal_deg[index],
                        min(self._upper[index] - self._limit_margin - nominal_deg[index],
                            value))
            self._follow[index] = value

    # -------------------------------------------------------------------- goal
    def _goal_callback(self, request):
        response = super()._goal_callback(request)
        if response == GoalResponse.ACCEPT:
            self._trim = [0.0] * 6
            self._follow = [0.0] * 6
            self._follow_object = ""
            self._follow_centre = None
            self._place_bin_id = ""
            self._place_target = None
            self._place_fix = [0.0] * 6
            self._place_z_ref = None
            self._place_z_ticks = 0
        return response

    # --------------------------------------------------------- placement fix
    def _place_error(self, want_z: bool):
        """World (dx, dy, dz) still needed to put the *block* on the slot."""
        pose = self._current_pose(self._follow_object)
        if pose is None or pose[3] < UPRIGHT_MIN_Z_M:
            return None
        dx = float(self._place_target[0]) - pose[1]
        dy = float(self._place_target[1]) - pose[2]
        span = math.hypot(dx, dy)
        if span > PLACE_MAX_SHIFT_M:
            scale = PLACE_MAX_SHIFT_M / span
            dx, dy = dx * scale, dy * scale
        dz = 0.0
        if want_z:
            dz = max(-PLACE_MAX_SHIFT_M,
                     min(PLACE_MAX_SHIFT_M, PLACE_OBJECT_Z_M - pose[3]))
        return dx, dy, dz

    def _measure_place_delta(self, above_nominal) -> None:
        """Seed the placement correction from the settled approach pose.

        The grasp sits ~13 mm below the jaw midpoint, the jaws can slip a few
        millimetres in transit, and the real pad midpoint sags a centimetre below
        the forward-kinematics prediction at this reach.  Commanding the jaw
        midpoint straight at the slot therefore drives the *block* into the
        floor, where the arm stalls, the trim saturates and the block is let go
        wherever the jaws happen to be.  The block-to-slot offset is measured
        here and used as the starting point of the closed loop below.
        """
        if self._place_target is None:
            return
        pose = self._current_pose(self._follow_object)
        phases = self.motion_config.get("bins", {}).get(self._place_bin_id, {})
        place_nominal = phases.get("place")
        if pose is None or pose[3] < UPRIGHT_MIN_Z_M or not place_nominal:
            return
        above_tool = kinematics.pad_mid(above_nominal, self._open, self._base_z)
        place_tool = kinematics.pad_mid(place_nominal, self._open, self._base_z)
        # Height the jaws must reach for the *block* to land on the floor: the
        # gap between the block and the jaw midpoint is carried down from here.
        error = [
            float(self._place_target[0]) - pose[1],
            float(self._place_target[1]) - pose[2],
            (above_tool[2] + PLACE_OBJECT_Z_M - pose[3]) - place_tool[2],
        ]
        span = math.hypot(error[0], error[1])
        if span > PLACE_MAX_SHIFT_M:
            scale = PLACE_MAX_SHIFT_M / span
            error[0], error[1] = error[0] * scale, error[1] * scale
        error[2] = max(-PLACE_MAX_SHIFT_M, min(PLACE_MAX_SHIFT_M, error[2]))
        self._place_fix = list(kinematics.xyz_correction(
            place_nominal, self._open, tuple(error),
            gain=1.0, limit_deg=PLACE_LIMIT_DEG, base_z=self._base_z,
        ))
        self._log(
            "place_correction",
            bin_id=self._place_bin_id,
            block_xyz=[round(value, 5) for value in (pose[1], pose[2], pose[3])],
            delta_m=[round(value, 5) for value in error],
            fix_deg=[round(value, 3) for value in self._place_fix],
        )

    def _limited_z_demand(self, dz: float) -> float:
        """Drop a downward demand that the block is not responding to.

        The block's resting height inside a region is 0.430, not the 0.425 of the
        bare table, because the regions sit on a 5 mm slab.  Asking for more depth
        than that never converges; the clamp is reached and the coupled solve
        drags the jaws sideways with it.  A downward demand that produces no
        downward progress for a second is therefore abandoned.
        """
        if dz >= 0.0:
            self._place_z_ref = None
            self._place_z_ticks = 0
            return dz
        pose = self._current_pose(self._follow_object)
        if pose is None:
            return dz
        if self._place_z_ref is None or pose[3] < self._place_z_ref - 0.0005:
            self._place_z_ref = pose[3]
            self._place_z_ticks = 0
            return dz
        self._place_z_ticks += 1
        if self._place_z_ticks >= int(round(PLACE_Z_STALL_S * self._rate_hz)):
            return 0.0
        return dz

    def _refresh_place(self, nominal, want_z: bool) -> None:
        """Integrate the residual block-to-slot error while a step is held.

        This is the loop that actually lands the block: the one-shot seed above
        is computed with the approach pose's Jacobian, so it is only approximately
        right once the arm has reached down, and the block keeps slipping in the
        jaws.  Integrating the *measured* block position closes both gaps.
        """
        if self._place_gain <= 0.0 or self._place_target is None:
            return
        error = self._place_error(want_z)
        if error is None:
            return
        horizontal = (error[0], error[1], 0.0)
        vertical = (0.0, 0.0, self._limited_z_demand(error[2]))
        # Solve the horizontal and the vertical demand separately.  One coupled
        # solve is cheaper but lets a vertical demand that cannot be satisfied
        # pull the horizontal correction off with it.
        step = [0.0] * 6
        for demand in (horizontal, vertical):
            if demand == (0.0, 0.0, 0.0):
                continue
            part = kinematics.xyz_correction(
                nominal, self._open, demand,
                gain=1.0, limit_deg=PLACE_LIMIT_DEG, base_z=self._base_z,
            )
            for index in range(6):
                step[index] += part[index]
        for index in range(6):
            value = self._place_fix[index] + step[index] * self._place_gain / self._rate_hz
            value = max(-PLACE_LIMIT_DEG, min(PLACE_LIMIT_DEG, value))
            value = max(self._lower[index] + self._limit_margin - nominal[index],
                        min(self._upper[index] - self._limit_margin - nominal[index],
                            value))
            self._place_fix[index] = value

    # ------------------------------------------------------------------- steps
    def _run_step(self, goal_handle, step, index: int, total: int) -> bool:
        nominal = tuple(float(value) for value in step.arm_deg)
        start_arm = tuple(float(value) for value in self._last_arm)
        start_gripper = float(self._last_gripper)
        target_gripper = float(step.gripper_rad)
        dt = 1.0 / self._rate_hz
        follow_here = (
            self._follow_gain > 0.0
            and (step.stage.startswith(FOLLOW_PREFIX) or step.stage in FOLLOW_EXTRA)
            and step.stage not in FOLLOW_SKIP
        )
        place_here = step.stage in PLACE_STAGES and self._place_target is not None
        place_want_z = step.stage in PLACE_Z_STAGES
        # The approach stage only seeds the correction (measured once it has
        # settled); the two steps that reach the floor integrate the residual.
        place_servo = place_here and step.stage != PLACE_ENTRY_STAGE

        def effective(reference):
            # The object-relative offset only exists while approaching, gripping
            # and closing.  Once the block is held it must be blended out, or it
            # would displace the transfer and the bin drop by the same amount.
            out = [float(value) for value in reference]
            if follow_here:
                for index in range(6):
                    out[index] += self._follow[index]
            if place_here:
                for index in range(6):
                    out[index] += self._place_fix[index]
            return tuple(out)

        # Phase 1: smooth ramp toward the target pose.  Every closed loop is
        # frozen here on purpose: the plant delivers only about a third of a
        # commanded joint change, so against a moving reference an integrator
        # accumulates the servo lag instead of the error and saturates.  The
        # compensation therefore only adapts while a step is held.
        ticks = max(1, int(round(step.duration_s * self._rate_hz)))
        for tick in range(1, ticks + 1):
            if goal_handle.is_cancel_requested:
                return False
            fraction = tick / ticks
            if follow_here:
                self._refresh_follow(nominal)
            target_arm = effective(nominal)
            arm = tuple(
                _smoothstep(start, target, fraction)
                for start, target in zip(start_arm, target_arm)
            )
            gripper = _smoothstep(start_gripper, target_gripper, fraction)
            self._publish_arm(arm)
            self._publish_gripper(gripper)
            self._publish_feedback(goal_handle, step.stage, (index + fraction) / total)
            self._last_arm = arm
            self._last_gripper = gripper
            time.sleep(dt)

        # Phase 2: hold the target.  Now the object-relative servo and the
        # joint trim are both live, and the step only completes once the joints
        # have actually arrived (or the timeout lapses).  The placement loop is
        # live here too: its reference is the slot, which does not move, so it
        # converges instead of winding up.
        hold_ticks = max(1, int(round(self._hold_timeout_s * self._rate_hz)))
        worst_error = float("inf")
        for _ in range(hold_ticks):
            if goal_handle.is_cancel_requested:
                return False
            if follow_here:
                self._refresh_follow(nominal)
            if place_servo:
                self._refresh_place(nominal, place_want_z)
            target_arm = effective(nominal)
            worst_error = self._integrate_trim(target_arm, dt)
            self._publish_arm(target_arm)
            self._publish_gripper(target_gripper)
            self._publish_feedback(goal_handle, step.stage, (index + 1.0) / total)
            self._last_arm = target_arm
            self._last_gripper = target_gripper
            if worst_error <= self._trim_tolerance and self._arm_settled():
                break
            time.sleep(dt)

        self._last_arm = effective(nominal)
        self._last_gripper = target_gripper
        self._publish_feedback(goal_handle, step.stage, (index + 1.0) / total)
        with self._joint_lock:
            sample = self._measured_arm
        measured = None if sample is None else [round(math.degrees(v), 3) for v in sample[1]]
        tool = self._measured_tool_centre()
        self._log(
            "stage",
            stage=step.stage,
            progress=(index + 1.0) / total,
            joint_error_deg=(None if worst_error == float("inf") else round(worst_error, 3)),
            trim_deg=[round(value, 2) for value in self._trim],
            follow_deg=[round(value, 3) for value in self._follow] if follow_here else None,
            measured_deg=measured,
            tool_xyz=None if tool is None else [round(value, 5) for value in tool],
            object_xyz=(
                None if self._current_pose(self._follow_object) is None
                else [round(self._current_pose(self._follow_object)[i], 5) for i in (1, 2, 3)]
            ),
        )
        if step.stage == PLACE_ENTRY_STAGE:
            # The approach pose has settled, so this is the moment to measure how
            # the block sits inside the jaws before the placement descent starts.
            self._measure_place_delta(nominal)
        return True

    # ------------------------------------------------------------------ target
    def _execute_callback(self, goal_handle):
        # Remember which block this goal is about, and where it started, so the
        # Cartesian servo only ever follows the intended object.
        nearest = self._nearest_object(goal_handle.request.grid_id)
        if nearest is not None:
            self._follow_object = nearest[0]
            self._follow_centre = (nearest[1][1], nearest[1][2])
        # Landing slot for the placement correction.
        self._place_bin_id = str(goal_handle.request.bin_id)
        self._place_target = self._bin_centres.get(self._place_bin_id)
        self._place_fix = [0.0] * 6
        self._place_z_ref = None
        self._place_z_ticks = 0
        return super()._execute_callback(goal_handle)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SixPickSortServer()
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
