"""Solve the six pick waypoints by inverse kinematics so the jaws straddle
each block dead centre, with a purely vertical descent.

Two defects in the hand-measured legacy waypoints are fixed here:

1. ``J1`` was an evenly spaced guess (33, 48, 76, 104, 132, 160) rather than the
   bearing to each block.  With ``J4 = 0`` and ``J6 = J1`` the tool centre lies
   in the arm's vertical plane at bearing ``J1``, so a wrong ``J1`` displaces
   the jaws sideways by ``r * sin(delta)`` -- 4-6 mm, which is more than the
   clearance the splayed 34 mm pads have around a 28 mm block.
2. The descent waypoints were not a vertical line, so the open jaws swept
   laterally through the block and knocked it over.

Now the tool centre (midpoint of the two rubber pads) is placed exactly on each
block centre at every level, and the levels differ only in height.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import yaml

# Use the same forward kinematics the action server runs, so a solved pose and
# the deployed compensation can never disagree.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src"
                            / "task3_six_sim" / "task3_six_sim"))
import kinematics

GRIDS = {
    "P1": (0.136387, 0.092863),
    "P2": (0.091310, 0.137432),
    "P3": (0.032755, 0.161716),
    "P4": (-0.030635, 0.162131),
    "P5": (-0.089503, 0.138615),
    "P6": (-0.135160, 0.094640),
}
PAD_CENTRE_Z = 0.426          # block spans 0.400..0.450 -> pads grip the middle
# Bin drop: the pads must release the block centred over the slot, with the
# block already standing on the bin floor (top 0.405) so it cannot topple.
PLACE_Z = 0.432
BINS = {
    "YELLOW_1": (0.106557, -0.047883),
    "YELLOW_2": (0.144057, -0.026233),
    "YELLOW_3": (0.144057, -0.069533),
    "GREEN_1": (-0.072883, -0.131557),
    "GREEN_2": (-0.035383, -0.109907),
    "GREEN_3": (-0.035383, -0.153207),
}
# Descent waypoints, relative to the pick height.  Two of them bracket the top of
# the block (0.450): the jaws pause just above it and again just below it, so the
# object-relative servo has a settled pose on both sides of the one transition
# where the pads' lower corners could clip the block's top edge.
LEVELS = [
    ("pick", 0.000),        # pad bottom 0.409, straddling the block
    ("descent_3", 0.021),   # pad bottom 0.430, 20 mm past the top
    ("descent_2", 0.051),   # pad bottom 0.460, 10 mm above the top
    ("descent_1", 0.115),   # pad bottom 0.524
    ("above", 0.150),       # transfer clearance
]

SRC = (Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "task3_six_sim"
       / "config" / "motion_points_six.yaml")
# Written next to this script so it can be diffed against the deployed config
# before being copied over.  Override with IK_OUT=... to write elsewhere.
OUT = Path(os.environ.get("IK_OUT") or
           Path(__file__).resolve().parent / "motion_ik.yaml")


def pad_mid(j1, q):
    return np.array(kinematics.pad_mid([j1, q[0], q[1], 0.0, q[2], j1], 0.14))


def ik(j1, target, seed, iters=600, damping=1e-4, step_max=2.5):
    """Damped least squares with a step clamp -- the wrist has singularities."""
    target = np.array(target, dtype=float)
    q = np.array(seed, dtype=float)
    best_q, best_r = q.copy(), float("inf")
    for _ in range(iters):
        mid = pad_mid(j1, q)
        err = target - mid
        residual = float(np.linalg.norm(err))
        if residual < best_r:
            best_q, best_r = q.copy(), residual
        if residual < 1e-7:
            break
        jac = np.zeros((3, 3))
        for k in range(3):
            delta = np.zeros(3)
            delta[k] = 1e-4
            jac[:, k] = (pad_mid(j1, q + delta) - mid) / 1e-4
        step = np.linalg.solve(jac.T @ jac + damping * np.eye(3), jac.T @ err)
        norm = np.linalg.norm(step)
        if norm > step_max:
            step *= step_max / norm
        q = q + step
    return best_q, best_r


def main():
    config = yaml.safe_load(SRC.read_text(encoding="utf-8"))
    for grid, (gx, gy) in GRIDS.items():
        j1 = math.degrees(math.atan2(gy, gx))
        legacy_pick = config["picks"][grid]["pick"]
        # Walk from the pick upwards so every level stays on one elbow branch.
        seed = (legacy_pick[1], legacy_pick[2], legacy_pick[4])
        solved = {}
        for name, dz in LEVELS:
            q, residual = ik(j1, (gx, gy, PAD_CENTRE_Z + dz), seed)
            if residual > 2e-4:
                raise SystemExit(f"{grid}.{name}: IK did not converge ({residual*1000:.3f} mm)")
            solved[name] = [round(float(j1), 4), round(float(q[0]), 4), round(float(q[1]), 4),
                            0.0, round(float(q[2]), 4), round(float(j1), 4)]
            seed = tuple(q)
            print(f"{grid} {name:10s} z={PAD_CENTRE_Z+dz:.3f}  J1={j1:+8.3f} J2={q[0]:+8.3f} "
                  f"J3={q[1]:+8.3f} J5={q[2]:+8.3f}   residual={residual*1000:.4f} mm")

        # continuity: no waypoint may demand a big joint reconfiguration
        order = [solved[n] for n, _ in reversed(LEVELS)]
        for a, b in zip(order, order[1:]):
            delta = max(abs(x - y) for x, y in zip((a[1], a[2], a[4]), (b[1], b[2], b[4])))
            if delta > 45.0:
                raise SystemExit(f"{grid}: descent branch jump of {delta:.1f} deg")

        # verify the solved pick really centres the jaws on the block
        left, right = (np.array(pad) for pad in
                       kinematics.pad_centres(solved["pick"], 0.14))
        mid = (left + right) / 2
        print(f"      verify pick: pad mid=({mid[0]:+.5f},{mid[1]:+.5f},{mid[2]:+.5f}) "
              f"block=({gx:+.5f},{gy:+.5f})  offset=({mid[0]-gx:+.5f},{mid[1]-gy:+.5f})  "
              f"jaw span={np.linalg.norm(left-right)*1000:.1f} mm  padBottom={mid[2]-0.017:.4f}")

        config["picks"][grid]["above"] = solved["above"]
        config["picks"][grid]["descent"] = [solved["descent_1"], solved["descent_2"],
                                            solved["descent_3"]]
        config["picks"][grid]["pick"] = solved["pick"]

    for name, (bin_x, bin_y) in BINS.items():
        j1 = math.degrees(math.atan2(bin_y, bin_x))
        legacy = config["bins"][name]
        seed = (legacy["place"][1], legacy["place"][2], legacy["place"][4])
        for level, z in (("place", PLACE_Z), ("above", PLACE_Z + 0.10)):
            q, residual = ik(j1, (bin_x, bin_y, z), seed)
            if residual > 2e-4:
                raise SystemExit(f"{name}.{level}: IK did not converge ({residual*1000:.3f} mm)")
            config["bins"][name][level] = [round(float(j1), 4), round(float(q[0]), 4),
                                           round(float(q[1]), 4), 0.0,
                                           round(float(q[2]), 4), round(float(j1), 4)]
            seed = tuple(q)
            print(f"{name:9s} {level:6s} z={z:.3f} J1={j1:+8.3f} J2={q[0]:+8.3f} "
                  f"J3={q[1]:+8.3f} J5={q[2]:+8.3f}  residual={residual*1000:.4f} mm")

    OUT.write_text(yaml.safe_dump(config, sort_keys=False, default_flow_style=None, width=200),
                   encoding="utf-8")
    print("\nwritten", OUT)


if __name__ == "__main__":
    main()
