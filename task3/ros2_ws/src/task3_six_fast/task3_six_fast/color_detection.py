"""Small, dependency-free colour detector for the fixed six-grid scene.

The simulator already publishes an overhead RGB image.  Since the pickup grids
do not move, object recognition only has to answer one question in a small ROI
around each grid: are the saturated pixels yellow or green?  Keeping this module
free of ROS and OpenCV makes the decision logic easy to test on the Jetson.
"""

from __future__ import annotations

import colorsys
import math
from collections import Counter
from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional, Sequence, Tuple


SUPPORTED_ENCODINGS = {
    "rgb8": (3, (0, 1, 2)),
    "bgr8": (3, (2, 1, 0)),
    "rgba8": (4, (0, 1, 2)),
    "bgra8": (4, (2, 1, 0)),
}


@dataclass(frozen=True)
class ColourDecision:
    colour: Optional[str]
    confidence: float
    yellow_pixels: int
    green_pixels: int


def project_grids(
    grids: Mapping[str, Sequence[float]],
    image_size: Sequence[int],
    horizontal_fov: float,
    camera_z: float,
    object_top_z: float,
    pixel_offset: Sequence[float] = (0.0, 0.0),
) -> Dict[str, Tuple[float, float]]:
    """Project world XY onto the fixed downward-looking Gazebo camera.

    The camera in ``task3_six_world.sdf`` is at the world origin in XY and has
    pitch +pi/2.  Ignition cameras look along local +X, so world +Y points to
    image left and world +X points to image up.
    """
    width, height = int(image_size[0]), int(image_size[1])
    depth = float(camera_z) - float(object_top_z)
    if width <= 0 or height <= 0 or depth <= 0.0:
        raise ValueError("invalid camera geometry")
    focal = width / (2.0 * math.tan(float(horizontal_fov) / 2.0))
    du, dv = float(pixel_offset[0]), float(pixel_offset[1])
    return {
        str(grid_id): (
            width / 2.0 - focal * float(xy[1]) / depth + du,
            height / 2.0 - focal * float(xy[0]) / depth + dv,
        )
        for grid_id, xy in grids.items()
    }


def _pixel_colour(red: int, green: int, blue: int) -> Optional[str]:
    maximum = max(red, green, blue)
    minimum = min(red, green, blue)
    if maximum < 55 or maximum == minimum:
        return None
    hue, saturation, value = colorsys.rgb_to_hsv(
        red / 255.0, green / 255.0, blue / 255.0
    )
    degrees = hue * 360.0
    if saturation < 0.35 or value < 0.22:
        return None
    if 38.0 <= degrees <= 75.0:
        return "YELLOW"
    if 75.0 < degrees <= 165.0:
        return "GREEN"
    return None


def classify_roi(
    data: bytes,
    width: int,
    height: int,
    step: int,
    encoding: str,
    centre: Sequence[float],
    half_size: int = 16,
    min_coloured_pixels: int = 24,
    min_confidence: float = 0.72,
) -> ColourDecision:
    """Classify one square image ROI as yellow, green, or unknown."""
    spec = SUPPORTED_ENCODINGS.get(str(encoding).lower())
    if spec is None:
        raise ValueError("unsupported image encoding: " + str(encoding))
    channels, order = spec
    width, height, step = int(width), int(height), int(step)
    if step < width * channels or len(data) < step * height:
        raise ValueError("image buffer is shorter than its dimensions")

    u, v = int(round(float(centre[0]))), int(round(float(centre[1])))
    radius = max(1, int(half_size))
    x0, x1 = max(0, u - radius), min(width, u + radius + 1)
    y0, y1 = max(0, v - radius), min(height, v + radius + 1)
    counts = Counter()
    view = memoryview(data)
    for y in range(y0, y1):
        row = y * step
        for x in range(x0, x1):
            base = row + x * channels
            colour = _pixel_colour(
                int(view[base + order[0]]),
                int(view[base + order[1]]),
                int(view[base + order[2]]),
            )
            if colour is not None:
                counts[colour] += 1

    yellow = int(counts["YELLOW"])
    green = int(counts["GREEN"])
    total = yellow + green
    if total < int(min_coloured_pixels):
        return ColourDecision(None, 0.0, yellow, green)
    colour = "YELLOW" if yellow >= green else "GREEN"
    confidence = max(yellow, green) / float(total)
    if confidence < float(min_confidence):
        colour = None
    return ColourDecision(colour, confidence, yellow, green)


def stable_colours(
    histories: Mapping[str, Iterable[Optional[str]]],
    confirm_frames: int,
) -> Optional[Dict[str, str]]:
    """Return all grid labels once each has the requested consecutive votes."""
    wanted = max(1, int(confirm_frames))
    result = {}
    for grid_id, values in histories.items():
        tail = list(values)[-wanted:]
        if len(tail) != wanted or tail[0] is None or len(set(tail)) != 1:
            return None
        result[str(grid_id)] = str(tail[0])
    return result


def build_sort_jobs(colours: Mapping[str, str]) -> Tuple[Tuple[str, str, str], ...]:
    """Assign each detected object to the next free slot of its colour."""
    used = {"YELLOW": 0, "GREEN": 0}
    jobs = []
    for grid_id in sorted(colours, key=lambda value: int(value.lstrip("Pp"))):
        colour = str(colours[grid_id]).upper()
        if colour not in used:
            raise ValueError(f"unknown colour at {grid_id}: {colour}")
        used[colour] += 1
        if used[colour] > 3:
            raise ValueError(f"more than three {colour.lower()} objects detected")
        jobs.append((str(grid_id), f"{colour}_{used[colour]}", colour))
    if used != {"YELLOW": 3, "GREEN": 3}:
        raise ValueError(
            "expected three yellow and three green objects, got "
            f"yellow={used['YELLOW']} green={used['GREEN']}"
        )
    return tuple(jobs)
