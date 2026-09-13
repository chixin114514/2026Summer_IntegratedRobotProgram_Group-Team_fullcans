"""Small freshness and measured-pose evidence helpers for Task3."""

from __future__ import annotations

import math
import re
from typing import Iterable, Optional, Sequence, Tuple


MAX_POSE_AGE_S = 0.5
POSE_SAMPLE = Tuple[float, float, float, float]
OBJECT_PREFIXES = ("orange_battery_", "green_can_")


def extract_model_name(frame_id: str) -> str:
    """Extract a target model from common Gazebo TF frame spellings."""
    parts = [part for part in re.split(r"::|/", str(frame_id or "")) if part]
    for part in parts:
        if part.startswith(OBJECT_PREFIXES):
            return part
    return ""


def frame_priority(frame_id: str, model_name: str) -> int:
    """Prefer a model-level frame over one of that model's link frames."""
    parts = [part for part in re.split(r"::|/", str(frame_id or "")) if part]
    try:
        model_index = parts.index(str(model_name))
    except ValueError:
        return 99
    return 0 if model_index == len(parts) - 1 else 1


def is_fresh(
    sample: Optional[Sequence[float]],
    now: float,
    max_age_s: float = MAX_POSE_AGE_S,
) -> bool:
    """Return true only for a finite sample no older than ``max_age_s``."""
    if sample is None or len(sample) < 4:
        return False
    try:
        timestamp = float(sample[0])
        current_time = float(now)
        age_limit = float(max_age_s)
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (timestamp, current_time, age_limit)):
        return False
    age = current_time - timestamp
    return 0.0 <= age <= age_limit


def _fresh_samples(
    samples: Iterable[Sequence[float]],
    now: float,
    max_age_s: float = MAX_POSE_AGE_S,
) -> list[POSE_SAMPLE]:
    result = []
    for sample in samples:
        if not is_fresh(sample, now, max_age_s):
            continue
        values = tuple(float(value) for value in sample[:4])
        if all(math.isfinite(value) for value in values):
            result.append(values)  # type: ignore[arg-type]
    return sorted(result, key=lambda value: value[0])


def estimate_speed(
    samples: Iterable[Sequence[float]],
    now: float,
    max_age_s: float = MAX_POSE_AGE_S,
) -> float:
    """Estimate the largest 3-D speed from fresh callback samples.

    An old latest sample or fewer than two fresh samples is inconclusive and
    therefore returns infinity rather than being treated as stationary.
    """
    ordered = sorted(
        (tuple(float(value) for value in sample[:4]) for sample in samples),
        key=lambda value: value[0],
    )
    if not ordered or not is_fresh(ordered[-1], now, max_age_s):
        return math.inf
    fresh = _fresh_samples(ordered, now, max_age_s)
    if len(fresh) < 2:
        return math.inf
    speeds = []
    for previous, current in zip(fresh[:-1], fresh[1:]):
        elapsed = current[0] - previous[0]
        if elapsed <= 1.0e-6:
            continue
        distance = math.sqrt(
            (current[1] - previous[1]) ** 2
            + (current[2] - previous[2]) ** 2
            + (current[3] - previous[3]) ** 2
        )
        speeds.append(distance / elapsed)
    return max(speeds, default=math.inf)


def placement_is_valid(
    pose: Optional[Sequence[float]],
    bin_xy: Sequence[float],
    samples: Iterable[Sequence[float]],
    now: float,
    max_xy_error_m: float = 0.045,
    z_min_m: float = 0.410,
    z_max_m: float = 0.445,
    max_speed_m_s: float = 0.03,
    max_age_s: float = MAX_POSE_AGE_S,
) -> bool:
    """Require fresh pose, settled height/location, and low measured speed."""
    if pose is None or len(pose) < 4 or len(bin_xy) < 2:
        return False
    if not is_fresh(pose, now, max_age_s):
        return False
    try:
        x, y, z = (float(pose[1]), float(pose[2]), float(pose[3]))
        bin_x, bin_y = (float(bin_xy[0]), float(bin_xy[1]))
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(value) for value in (x, y, z, bin_x, bin_y)):
        return False
    distance = math.hypot(x - bin_x, y - bin_y)
    speed = estimate_speed(samples, now, max_age_s)
    return (
        distance <= float(max_xy_error_m)
        and float(z_min_m) <= z <= float(z_max_m)
        and speed <= float(max_speed_m_s)
    )
