import math
import unittest
from pathlib import Path


TASK3_ROOT = Path(__file__).resolve().parents[4]
CONFIG_PATH = (
    TASK3_ROOT
    / "ros2_ws"
    / "src"
    / "task3_sim"
    / "config"
    / "motion_points.yaml"
)
SCENE_CONFIG_PATH = CONFIG_PATH.parent / "scene.yaml"


EXPECTED_GRIDS = {
    "P1": [0.13213, 0.06812],
    "P2": [0.08037, -0.12506],
    "P3": [-0.08037, 0.12506],
    "P4": [-0.13213, -0.06812],
}
EXPECTED_BINS = {
    "BIN_A": [0.12557, -0.03365],
    "BIN_B": [-0.11782, 0.05494],
}
EXPECTED_LIMITS = {
    "lower": [-160, -75, -175, -155, -115, -180],
    "upper": [160, 120, 65, 155, 115, 180],
}


def _load_yaml(path):
    try:
        import yaml
    except ModuleNotFoundError:
        import ast

        result = {}
        stack = [(0, result)]
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            if not raw_line.strip() or raw_line.lstrip().startswith("#"):
                continue
            indent = len(raw_line) - len(raw_line.lstrip())
            key, raw_value = raw_line.strip().split(":", 1)
            while stack and indent < stack[-1][0]:
                stack.pop()
            parent = stack[-1][1]
            if not raw_value.strip():
                parent[key] = {}
                stack.append((indent + 2, parent[key]))
            else:
                try:
                    parent[key] = ast.literal_eval(raw_value.strip())
                except (ValueError, SyntaxError):
                    parent[key] = raw_value.strip()
        return result
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class MotionConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not CONFIG_PATH.is_file():
            raise AssertionError(f"missing motion config: {CONFIG_PATH}")
        if not SCENE_CONFIG_PATH.is_file():
            raise AssertionError(f"missing scene config: {SCENE_CONFIG_PATH}")
        cls.config = _load_yaml(CONFIG_PATH)
        cls.scene = _load_yaml(SCENE_CONFIG_PATH)

    def test_fixed_scene_centres_are_reachability_checked_values(self):
        self.assertEqual(self.scene["grids"], EXPECTED_GRIDS)
        self.assertEqual(self.scene["bins"], EXPECTED_BINS)

    def test_all_fixed_poses_have_six_finite_limited_joint_values(self):
        limits = self.config["joint_limits_deg"]
        self.assertEqual(limits, EXPECTED_LIMITS)
        poses = {"home": self.config["home"]}
        poses.update(
            {
                f"{grid}.{phase}": pose
                for grid, phases in self.config["picks"].items()
                for phase, pose in phases.items()
                if phase != "descent"
            }
        )
        poses.update(
            {
                f"{grid}.descent_{index}": pose
                for grid, phases in self.config["picks"].items()
                for index, pose in enumerate(phases.get("descent", []))
            }
        )
        poses.update(
            {
                f"{bin_id}.{phase}": pose
                for bin_id, phases in self.config["bins"].items()
                for phase, pose in phases.items()
            }
        )
        self.assertIn("P1.above", poses)
        self.assertIn("P1.pick", poses)
        self.assertIn("BIN_A.above", poses)
        self.assertIn("BIN_A.place", poses)
        for name, pose in poses.items():
            self.assertEqual(len(pose), 6, name)
            for index, value in enumerate(pose):
                self.assertTrue(math.isfinite(float(value)), name)
                self.assertGreaterEqual(value, limits["lower"][index], name)
                self.assertLessEqual(value, limits["upper"][index], name)

    def test_p1_uses_measured_plus_y_gripper_center_correction(self):
        self.assertEqual(
            self.config["picks"]["P1"]["above"],
            [33.3624, 31.7225, -43.2093, 0.0, 101.4797, 33.3624],
        )
        self.assertEqual(
            self.config["picks"]["P1"]["pick"],
            [33.3541, 48.1151, -5.5796, -0.0058, 47.4603, 33.3584],
        )
        self.assertEqual(
            self.config["picks"]["P1"]["descent"],
            [
                [33.3627, 30.8631, -20.9472, 0.0, 80.0875, 33.3627],
                [33.3626, 37.6076, -9.1040, 0.0, 61.4978, 33.3627],
            ],
        )

    def test_common_heights_gripper_values_and_motion_rate_are_defined(self):
        self.assertEqual(set(self.config["picks"]), set(EXPECTED_GRIDS))
        self.assertEqual(set(self.config["bins"]), set(EXPECTED_BINS))
        self.assertEqual(self.config["heights"]["pickup_z"], self.config["heights"]["place_z"])
        self.assertEqual(self.config["gripper"]["open_rad"], 0.14)
        self.assertEqual(self.config["gripper"]["closed_rad"], -0.50)
        self.assertEqual(self.config["motion"]["command_rate_hz"], 20)
        self.assertEqual(
            self.config["motion"]["durations_s"],
            {
                "home": 2.0,
                "transfer": 2.0,
                "vertical": 1.2,
                "gripper": 0.8,
                "settle": 1.0,
                "descent_segment": 0.6,
            },
        )


if __name__ == "__main__":
    unittest.main()
