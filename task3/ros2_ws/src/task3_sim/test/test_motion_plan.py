import ast
import sys
import unittest
from pathlib import Path


TASK3_ROOT = Path(__file__).resolve().parents[4]
TASK3_PACKAGE = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim"
CONFIG_PATH = TASK3_PACKAGE / "config" / "motion_points.yaml"
sys.path.insert(0, str(TASK3_PACKAGE))


def _load_yaml(path):
    try:
        import yaml
    except ModuleNotFoundError:
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


class MotionPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not CONFIG_PATH.is_file():
            raise AssertionError(f"missing motion config: {CONFIG_PATH}")
        cls.config = _load_yaml(CONFIG_PATH)
        try:
            from task3_sim.motion_plan import build_sequence, validate_goal
        except ModuleNotFoundError as exc:
            raise AssertionError("motion_plan.py is required") from exc
        cls.build_sequence = staticmethod(build_sequence)
        cls.validate_goal = staticmethod(validate_goal)

    def test_validate_goal_accepts_known_grid_and_bin(self):
        self.assertIsNone(self.validate_goal("P1", "BIN_A", self.config))
        self.assertIsNone(self.validate_goal("P4", "BIN_B", self.config))

    def test_validate_goal_rejects_unknown_identifiers_before_motion(self):
        with self.assertRaises(ValueError):
            self.validate_goal("P5", "BIN_A", self.config)
        with self.assertRaises(ValueError):
            self.validate_goal("P1", "BIN_C", self.config)

    def test_build_sequence_has_fixed_order_and_requested_poses_only(self):
        sequence = self.build_sequence("P1", "BIN_A", self.config)
        self.assertEqual(
            [step.stage for step in sequence],
            [
                "OPEN",
                "PICK_ABOVE",
                "PICK",
                "CLOSE",
                "LIFT",
                "BIN_ABOVE",
                "BIN_PLACE",
                "RELEASE",
                "RETREAT",
                "HOME",
            ],
        )
        for step in sequence:
            self.assertEqual(len(step.arm_deg), 6, step.stage)
        allowed = {
            tuple(self.config["home"]),
            tuple(self.config["picks"]["P1"]["above"]),
            tuple(self.config["picks"]["P1"]["pick"]),
            tuple(self.config["bins"]["BIN_A"]["above"]),
            tuple(self.config["bins"]["BIN_A"]["place"]),
        }
        self.assertTrue(all(tuple(step.arm_deg) in allowed for step in sequence))
        self.assertEqual(sequence[0].arm_deg, tuple(self.config["home"]))
        self.assertEqual(
            sequence[1].arm_deg,
            tuple(self.config["picks"]["P1"]["above"]),
        )
        self.assertEqual(sequence[0].gripper_rad, self.config["gripper"]["open_rad"])
        self.assertEqual(sequence[3].gripper_rad, self.config["gripper"]["closed_rad"])
        self.assertEqual(sequence[-1].arm_deg, tuple(self.config["home"]))


if __name__ == "__main__":
    unittest.main()
