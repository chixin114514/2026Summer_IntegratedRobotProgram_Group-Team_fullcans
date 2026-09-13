"""Fixed-point motion sequence construction for the Task3 action server."""

from dataclasses import dataclass
from typing import Mapping, Tuple


@dataclass(frozen=True)
class MotionStep:
    stage: str
    arm_deg: Tuple[float, ...]
    gripper_rad: float
    duration_s: float


def validate_goal(grid_id: str, bin_id: str, config: Mapping) -> None:
    """Reject identifiers that do not have a configured fixed pose."""
    picks = config.get("picks", {})
    bins = config.get("bins", {})
    if grid_id not in picks:
        raise ValueError(f"unknown grid_id: {grid_id}")
    if bin_id not in bins:
        raise ValueError(f"unknown bin_id: {bin_id}")
    if not isinstance(picks[grid_id], Mapping) or not isinstance(bins[bin_id], Mapping):
        raise ValueError("fixed motion poses are malformed")
    for phase in ("above", "pick"):
        if phase not in picks[grid_id] or len(picks[grid_id][phase]) != 6:
            raise ValueError(f"missing six-joint pose: {grid_id}.{phase}")
    if "descent" in picks[grid_id]:
        descent = picks[grid_id]["descent"]
        if not isinstance(descent, (list, tuple)):
            raise ValueError(f"descent must be a list: {grid_id}")
        for index, pose in enumerate(descent, start=1):
            if not isinstance(pose, (list, tuple)) or len(pose) != 6:
                raise ValueError(f"missing six-joint pose: {grid_id}.descent_{index}")
    if "approach" in picks[grid_id]:
        approach = picks[grid_id]["approach"]
        if not isinstance(approach, (list, tuple)):
            raise ValueError(f"approach must be a list: {grid_id}")
        for index, pose in enumerate(approach, start=1):
            if not isinstance(pose, (list, tuple)) or len(pose) != 6:
                raise ValueError(f"missing six-joint pose: {grid_id}.approach_{index}")
    if "press" in picks[grid_id] and len(picks[grid_id]["press"]) != 6:
        raise ValueError(f"missing six-joint pose: {grid_id}.press")
    for phase in ("above", "place"):
        if phase not in bins[bin_id] or len(bins[bin_id][phase]) != 6:
            raise ValueError(f"missing six-joint pose: {bin_id}.{phase}")
    if len(config.get("home", ())) != 6:
        raise ValueError("missing six-joint home pose")
    if len(config.get("transfer_clearance", ())) != 6:
        raise ValueError("missing six-joint transfer clearance pose")


def build_sequence(grid_id: str, bin_id: str, config: Mapping) -> list[MotionStep]:
    """Build the one-object fixed-point sequence in execution order."""
    validate_goal(grid_id, bin_id, config)
    pick = config["picks"][grid_id]
    target = config["bins"][bin_id]
    home = tuple(float(value) for value in config["home"])
    pick_above = tuple(float(value) for value in pick["above"])
    pick_pose = tuple(float(value) for value in pick["pick"])
    approach_poses = pick.get("approach", ())
    descent_poses = pick.get("descent", ())
    bin_above = tuple(float(value) for value in target["above"])
    bin_place = tuple(float(value) for value in target["place"])
    transfer_clearance = tuple(
        float(value) for value in config["transfer_clearance"]
    )

    def clearance_at(reference):
        """Keep the loaded arm high while adopting a source/target bearing."""
        pose = list(transfer_clearance)
        pose[0] = reference[0]
        pose[5] = reference[5]
        return tuple(pose)

    source_clearance = clearance_at(pick_above)
    target_clearance = clearance_at(bin_above)

    gripper = config["gripper"]
    opened = float(gripper["open_rad"])
    closed = float(gripper["closed_rad"])
    durations = config["motion"]["durations_s"]

    def step(stage, arm_deg, gripper_rad, duration_key):
        return MotionStep(
            stage=stage,
            arm_deg=tuple(float(value) for value in arm_deg),
            gripper_rad=float(gripper_rad),
            duration_s=float(durations[duration_key]),
        )

    sequence = [step("OPEN", home, opened, "gripper")]
    sequence.extend(
        step(
            f"PICK_APPROACH_{index}",
            pose,
            opened,
            "transfer",
        )
        for index, pose in enumerate(approach_poses, start=1)
    )
    sequence.append(step("PICK_ABOVE", pick_above, opened, "transfer"))
    sequence.extend(
        step(
            f"PICK_DESCEND_{index}",
            pose,
            opened,
            "descent_segment",
        )
        for index, pose in enumerate(descent_poses, start=1)
    )
    pick_steps = [step("PICK", pick_pose, opened, "vertical")]
    close_pose = pick_pose
    press_pose = pick.get("press")
    if press_pose is not None:
        # Optional contact press.  The arm's position loops are soft, so the
        # tool lands in a path-dependent band around the pick pose -- wider
        # than the jaw opening.  Driving the open jaws a little below the pick
        # pose makes them settle onto the table surface beside the object,
        # which turns the grasp height into a repeatable hard constraint; the
        # jaws then close symmetrically over the object's whole height.
        # Without the key the legacy sequence is unchanged.
        press_pose = tuple(float(value) for value in press_pose)
        pick_steps.append(
            step(
                "PRESS",
                press_pose,
                opened,
                "press" if "press" in durations else "vertical",
            )
        )
        close_pose = press_pose
    if "pick_hold" in durations:
        pick_steps.append(step("PICK_HOLD", pick_pose, opened, "pick_hold"))
    pick_steps.append(step("CLOSE", close_pose, closed, "gripper"))
    sequence.extend(pick_steps)
    if descent_poses:
        sequence.extend(
            step(
                f"LIFT_ASCEND_{index}",
                pose,
                closed,
                "descent_segment",
            )
            for index, pose in enumerate(reversed(descent_poses), start=1)
        )
        lift_duration = "descent_segment"
    else:
        lift_duration = "vertical"
    sequence.extend(
        [
            step("LIFT", pick_above, closed, lift_duration),
            step("TRANSFER_LIFT", source_clearance, closed, "vertical"),
            step("TRANSFER_ROTATE", target_clearance, closed, "transfer"),
            step("BIN_ABOVE", bin_above, closed, "vertical"),
            step("BIN_PLACE", bin_place, closed, "vertical"),
            step("RELEASE", bin_place, opened, "gripper"),
            step("RETREAT", bin_above, opened, "vertical"),
            step("HOME", home, opened, "home"),
        ]
    )
    return sequence
