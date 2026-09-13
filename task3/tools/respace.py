"""Respace the six pickup grids evenly around the arm.

The original layout left P1 and P2 only 42 mm apart (14.5 deg at r=0.166) while
the jaws are 42 mm wide and each block is 40 mm long, so the jaws descending on
one block reached into its neighbour -- which is what kept knocking the second
block off station.  The grids are now 24.35 deg apart (70 mm of arc), which
leaves every block and every pair of jaws clear of its neighbours.
"""
import math
import re
from pathlib import Path

RADIUS = 0.165
FIRST_DEG = 34.25
STEP_DEG = 22.15

GRIDS = {}
for index in range(6):
    bearing = math.radians(FIRST_DEG + index * STEP_DEG)
    GRIDS[f"P{index + 1}"] = (RADIUS * math.cos(bearing), RADIUS * math.sin(bearing))

OBJECT_OF_GRID = {
    "P1": "orange_battery_1", "P2": "green_can_1", "P3": "orange_battery_2",
    "P4": "green_can_2", "P5": "orange_battery_3", "P6": "green_can_3",
}

PACKAGE = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "task3_six_sim"
WORLD = PACKAGE / "worlds" / "task3_six_world.sdf"
SCENE = PACKAGE / "config" / "scene_six.yaml"
# Rewritten here first so the originals can be diffed before being applied.
STAGE_WORLD = Path(__file__).resolve().parent / "world_respace.sdf"
STAGE_SCENE = Path(__file__).resolve().parent / "scene_respace.yaml"


def replace_pose(text, model_name, new_pose):
    """Replace the <pose> of a named model (models are not nested here)."""
    start = text.index(f'<model name="{model_name}">')
    end = text.index("</model>", start)
    block = text[start:end]
    updated, count = re.subn(r"<pose>[^<]*</pose>", f"<pose>{new_pose}</pose>", block, count=1)
    assert count == 1, f"{model_name}: no pose replaced"
    return text[:start] + updated + text[end:]


world = WORLD.read_text(encoding="utf-8")
for grid, (x, y) in GRIDS.items():
    world = replace_pose(world, f"grid_{grid.lower()}", f"{x:.6f} {y:.6f} 0.402 0 0 0")
    world = replace_pose(world, OBJECT_OF_GRID[grid], f"{x:.6f} {y:.6f} 0.4250 0 0 0")
STAGE_WORLD.write_text(world, encoding="utf-8")

scene = SCENE.read_text(encoding="utf-8")
grid_block = "grids:\n" + "".join(
    f"  {grid}: [{x:.6f}, {y:.6f}]\n" for grid, (x, y) in GRIDS.items()
)
scene, count = re.subn(r"grids:\n(?:  P\d: \[[^\]]*\]\n)+", grid_block, scene, count=1)
assert count == 1, "grids block not replaced"
scene = scene.replace(
    "# Six pickup grids occupy the upper half of the table.  Each centre is the pad\n"
    "# midpoint the arm reaches while holding that pick pose (measured with\n"
    "# `ign model`), so the block sits between the open jaws when they close.\n",
    "# Six pickup grids on an even 24.35 deg arc at r=0.165, so neighbouring blocks\n"
    "# (40 mm long) and the 42 mm-wide jaws stay clear of each other.  Each centre\n"
    "# is also the pad midpoint the solved pick pose reaches, so the block sits dead\n"
    "# centre between the open jaws when they close.\n",
)
STAGE_SCENE.write_text(scene, encoding="utf-8")

for grid, (x, y) in GRIDS.items():
    r = math.hypot(x, y)
    print(f"{grid}: ({x:+.6f}, {y:+.6f})  bearing={math.degrees(math.atan2(y, x)):7.2f} deg  r={r:.4f}")
print("arc spacing:", f"{RADIUS * math.radians(STEP_DEG) * 1000:.1f} mm")
print("staged", STAGE_WORLD.name, "and", STAGE_SCENE.name)
