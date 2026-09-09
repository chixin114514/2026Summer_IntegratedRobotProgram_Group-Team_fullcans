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
    for phase in ("above", "place"):
        if phase not in bins[bin_id] or len(bins[bin_id][phase]) != 6:
            raise ValueError(f"missing six-joint pose: {bin_id}.{phase}")
    if len(config.get("home", ())) != 6:
        raise ValueError("missing six-joint home pose")


def build_sequence(grid_id: str, bin_id: str, config: Mapping) -> list[MotionStep]:
    """Build the one-object fixed-point sequence in execution order."""
    validate_goal(grid_id, bin_id, config)
    pick = config["picks"][grid_id]
    target = config["bins"][bin_id]
    home = tuple(float(value) for value in config["home"])
    pick_above = tuple(float(value) for value in pick["above"])
    pick_pose = tuple(float(value) for value in pick["pick"])
    bin_above = tuple(float(value) for value in target["above"])
    bin_place = tuple(float(value) for value in target["place"])

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

    return [
        step("OPEN", home, opened, "gripper"),
        step("PICK_ABOVE", pick_above, opened, "transfer"),
        step("PICK", pick_pose, opened, "vertical"),
        step("CLOSE", pick_pose, closed, "gripper"),
        step("LIFT", pick_above, closed, "vertical"),
        step("BIN_ABOVE", bin_above, closed, "transfer"),
        step("BIN_PLACE", bin_place, closed, "vertical"),
        step("RELEASE", bin_place, opened, "gripper"),
        step("RETREAT", bin_above, opened, "vertical"),
        step("HOME", home, opened, "home"),
    ]
