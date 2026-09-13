"""Compare kinematics.pad_mid(measured_deg) with the sim's real pad midpoint."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src"
                            / "task3_six_sim" / "task3_six_sim"))
import kinematics

CASES = [
    # stage, measured_deg (from jsonl), nominal_deg, real tool_xyz
    ("LIFT", [34.277, 6.638, -19.824, 0.009, 74.836, 34.25],
     [34.2501, 6.4652, -19.9175, 0.0, 74.8234, 34.2501], [0.1413, 0.09672, 0.56456]),
    ("DESCEND_3", [29.816, 44.605, -5.949, 0.01, 67.367, 29.816],
     [34.2501, 46.738, -13.2258, 0.0, 56.8348, 34.2501], [0.11144, 0.06454, 0.4435]),
    ("DESCEND_2", [29.805, 33.686, -5.151, -0.002, 74.278, 29.815],
     [34.2501, 35.7984, -12.4739, 0.0, 63.7274, 34.2501], [0.11232, 0.06485, 0.47072]),
    ("OPEN", [0.0, -0.171, -20.076, -0.005, 110.009, 0.004],
     [0.0, 0.0, -20.0, 0.0, 110.0, 0.0], [0.09982, 3e-05, 0.57053]),
]

print(f"{'stage':12s} {'fk(measured)':38s} {'real':34s} {'fk-nominal':38s} err(mm)")
for stage, meas, nom, real in CASES:
    fk_m = kinematics.pad_mid(meas, 0.14, 0.40)
    fk_n = kinematics.pad_mid(nom, 0.14, 0.40)
    d = [(fk_m[i] - real[i]) * 1000 for i in range(3)]
    print(f"{stage:12s} {str([round(v,5) for v in fk_m]):38s} "
          f"{str([round(v,5) for v in real]):34s} {str([round(v,5) for v in fk_n]):38s} "
          f"{[round(v,1) for v in d]}")
