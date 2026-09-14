import json
import unittest

from fixed_joint_sorter.detection import classify_initial_layout, parse_message
from fixed_joint_sorter.robot import JointRobot


class CoreTests(unittest.TestCase):
    def test_initial_layout_left_column_then_right_column(self):
        payload = json.dumps([
            {"class_id": 0, "class_name": "charger", "confidence": .9, "bbox": {"x1": 20, "y1": 210, "x2": 60, "y2": 250}},
            {"class_id": 1, "class_name": "stapler_box", "confidence": .9, "bbox": {"x1": 320, "y1": 110, "x2": 360, "y2": 150}},
            {"class_id": 1, "class_name": "stapler_box", "confidence": .9, "bbox": {"x1": 20, "y1": 10, "x2": 60, "y2": 50}},
            {"class_id": 0, "class_name": "charger", "confidence": .9, "bbox": {"x1": 320, "y1": 210, "x2": 360, "y2": 250}},
            {"class_id": 1, "class_name": "stapler_box", "confidence": .9, "bbox": {"x1": 20, "y1": 110, "x2": 60, "y2": 150}},
            {"class_id": 0, "class_name": "charger", "confidence": .9, "bbox": {"x1": 320, "y1": 10, "x2": 360, "y2": 50}},
        ])
        frames = [parse_message(payload) for _ in range(5)]
        result = classify_initial_layout(frames, {"charger", "stapler_box"}, .7, 5)
        self.assertEqual(result, {
            "P1": "stapler_box", "P2": "stapler_box", "P3": "charger",
            "P4": "charger", "P5": "stapler_box", "P6": "charger",
        })

    def test_open_loop_does_not_read_coords(self):
        class Client:
            def __init__(self):
                self.sent = []
            def send_coords(self, target, speed, _async=True):
                self.sent.append(target)
                return -1
            def get_angles(self):
                raise AssertionError("open-loop controller must not call get_angles")
        client = Client()
        cfg = {"robot": {"arm_speed": 20, "command_interval_s": 0.0}}
        robot = JointRobot(cfg, logger=lambda _: None, client=client)
        robot.move([10, 20, 30, 40, 50, 60], "test")
        self.assertEqual(client.sent, [[10.0, 20.0, 30.0, 40.0, 50.0, 60.0]])


if __name__ == "__main__":
    unittest.main()
