import json
from pathlib import Path
import unittest
from unittest.mock import call, patch

from fixed_joint_sorter.detection import classify_initial_layout, parse_message
from fixed_joint_sorter.config import load_config
import fixed_joint_sorter.robot as robot_module
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
            def set_fresh_mode(self, mode):
                self.mode = mode
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

    def test_sets_interpolation_queue_mode_before_first_move(self):
        class Client:
            def __init__(self):
                self.calls = []

            def set_fresh_mode(self, mode):
                self.calls.append(("set_fresh_mode", mode))

            def send_coords(self, target, speed, _async=True):
                self.calls.append(("send_coords", target, speed, _async))

        client = Client()
        cfg = {"robot": {"arm_speed": 20, "command_interval_s": 0.0}}
        robot = JointRobot(cfg, logger=lambda _: None, client=client)
        robot.move([1, 2, 3, 4, 5, 6], "test")

        self.assertEqual(client.calls, [
            ("set_fresh_mode", 0),
            ("send_coords", [1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 20, True),
        ])

    def test_robot_logs_every_low_level_command(self):
        class Client:
            def set_fresh_mode(self, mode):
                return "mode-ack"

            def send_coords(self, target, speed, _async=True):
                return "move-ack"

            def set_gripper_state(self, flag, speed):
                return "gripper-ack"

            def stop(self):
                return "stop-ack"

        logs = []
        cfg = {
            "robot": {
                "arm_speed": 20,
                "gripper_speed": 100,
                "command_interval_s": 0.0,
                "gripper_settle_s": 0.0,
            }
        }
        robot = JointRobot(cfg, logger=logs.append, client=Client())
        robot.move([1, 2, 3, 4, 5, 6], "P1_approach")
        robot.gripper(close=True)
        robot.stop()

        self.assertEqual(logs, [
            "queue_mode: client.set_fresh_mode(0)",
            "P1_approach: client.send_coords(coords=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=20, _async=True)",
            "gripper: client.set_gripper_state(flag=1, speed=100)",
            "stop: client.stop()",
        ])

    def test_next_command_delay_applies_after_move_and_gripper(self):
        class Client:
            def set_fresh_mode(self, mode):
                return None

            def send_coords(self, target, speed, _async=True):
                return None

            def set_gripper_state(self, flag, speed):
                return None

        cfg = {
            "robot": {
                "arm_speed": 20,
                "gripper_speed": 100,
                "next_command_delay_s": 2.5,
                "command_interval_s": 0.1,
                "gripper_settle_s": 0.2,
            }
        }
        robot = JointRobot(cfg, logger=lambda _: None, client=Client())

        with patch.object(robot_module.time, "sleep") as sleep:
            robot.move([1, 2, 3, 4, 5, 6], "test_move")
            robot.gripper(close=True, label="test_gripper")

        self.assertEqual(sleep.call_args_list, [call(2.5), call(2.5)])

    def test_each_bin_reuses_its_own_drop_point_without_duplicate_slots(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "fixed_sorter.yaml"
        cfg = load_config(config_path)
        self.assertNotIn("drop_point", cfg)
        points = {}
        for bin_name, bin_cfg in cfg["bins"].items():
            self.assertNotIn("slots", bin_cfg)
            self.assertEqual(bin_cfg["expected_count"], 3)
            self.assertEqual(
                set(bin_cfg["drop_point"]),
                {"approach_angles", "place_angles", "retreat_angles"},
            )
            points[bin_name] = tuple(
                tuple(bin_cfg["drop_point"][phase])
                for phase in ("approach_angles", "place_angles", "retreat_angles")
            )
        self.assertEqual(len(points), 2)
        self.assertEqual(len(set(points.values())), 2)

    def test_pick_place_cycle_follows_requested_order(self):
        cycle = getattr(robot_module, "run_pick_place_cycle", None)
        self.assertIsNotNone(cycle)
        if cycle is None:
            return

        class FakeRobot:
            def __init__(self):
                self.events = []

            def gripper(self, close, label=None):
                self.events.append(("gripper", close, label))

            def move(self, target, label):
                self.events.append(("move", tuple(target), label))

        station = {
            "approach_angles": [1, 2, 3, 4, 5, 6],
            "pick_angles": [7, 8, 9, 10, 11, 12],
            "retreat_angles": [13, 14, 15, 16, 17, 18],
        }
        drop_point = {
            "approach_angles": [19, 20, 21, 22, 23, 24],
            "place_angles": [25, 26, 27, 28, 29, 30],
            "retreat_angles": [31, 32, 33, 34, 35, 36],
        }
        home = [37, 38, 39, 40, 41, 42]
        robot = FakeRobot()

        cycle(robot, station, drop_point, home, "P1")

        self.assertEqual(robot.events, [
            ("gripper", True, "P1_close_before_approach"),
            ("move", tuple(station["approach_angles"]), "P1_approach"),
            ("gripper", False, "P1_open_before_pick"),
            ("move", tuple(station["pick_angles"]), "P1_pick"),
            ("gripper", True, "P1_close_at_pick"),
            ("move", tuple(station["retreat_angles"]), "P1_retreat"),
            ("move", tuple(drop_point["place_angles"]), "P1_drop_place"),
            ("gripper", False, "P1_open_at_drop"),
            ("move", tuple(home), "P1_return_home"),
        ])


if __name__ == "__main__":
    unittest.main()
