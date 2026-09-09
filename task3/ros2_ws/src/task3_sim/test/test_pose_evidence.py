import math
import sys
import unittest
from pathlib import Path


TASK3_ROOT = Path(__file__).resolve().parents[4]
TASK3_PACKAGE = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim"
sys.path.insert(0, str(TASK3_PACKAGE))


class PoseEvidenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from task3_sim.pose_evidence import (
                estimate_speed,
                extract_model_name,
                frame_priority,
                is_fresh,
                placement_is_valid,
            )
        except ModuleNotFoundError as exc:
            raise AssertionError("pose_evidence.py is required") from exc
        cls.estimate_speed = staticmethod(estimate_speed)
        cls.extract_model_name = staticmethod(extract_model_name)
        cls.frame_priority = staticmethod(frame_priority)
        cls.is_fresh = staticmethod(is_fresh)
        cls.placement_is_valid = staticmethod(placement_is_valid)

    def test_old_pose_is_rejected_by_freshness_and_placement(self):
        now = 100.0
        old = (now - 0.51, 0.12557, -0.03365, 0.420)
        self.assertFalse(self.is_fresh(old, now))
        self.assertFalse(
            self.placement_is_valid(
                old,
                (0.12557, -0.03365),
                [old, (now - 0.50, 0.12557, -0.03365, 0.420)],
                now,
            )
        )

    def test_moving_samples_cannot_be_classified_as_low_speed(self):
        samples = [
            (99.8, 0.100, -0.030, 0.420),
            (99.9, 0.110, -0.030, 0.420),
            (100.0, 0.120, -0.030, 0.420),
        ]
        speed = self.estimate_speed(samples, 100.0)
        self.assertGreater(speed, 0.03)
        self.assertFalse(
            self.placement_is_valid(
                samples[-1], (0.120, -0.030), samples, 100.0
            )
        )

    def test_fresh_static_samples_pass_placement_evidence(self):
        samples = [
            (99.8, 0.12557, -0.03365, 0.420),
            (99.9, 0.12557, -0.03365, 0.420),
            (100.0, 0.12557, -0.03365, 0.420),
        ]
        self.assertTrue(self.is_fresh(samples[-1], 100.0))
        self.assertTrue(
            self.placement_is_valid(
                samples[-1], (0.12557, -0.03365), samples, 100.0
            )
        )

    def test_model_name_is_extracted_from_supported_frame_formats(self):
        cases = {
            "orange_battery_1::body": "orange_battery_1",
            "green_can_2/body": "green_can_2",
            "world/orange_battery_3/body": "orange_battery_3",
            "world/green_can_4/body": "green_can_4",
        }
        for frame_id, expected in cases.items():
            self.assertEqual(self.extract_model_name(frame_id), expected)

    def test_frame_priority_prefers_model_frame_over_link_frame(self):
        model = "orange_battery_1"
        cases = {
            "orange_battery_1": 0,
            "world/orange_battery_1": 0,
            "orange_battery_1/body": 1,
            "world/orange_battery_1/body": 1,
        }
        for frame_id, expected in cases.items():
            self.assertEqual(self.frame_priority(frame_id, model), expected)


if __name__ == "__main__":
    unittest.main()
