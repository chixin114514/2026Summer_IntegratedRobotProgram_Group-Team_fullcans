import ast
import sys
import unittest
from pathlib import Path
from xml.etree import ElementTree


PACKAGE = Path(__file__).resolve().parents[1]
WORLD_PATH = PACKAGE / "worlds" / "task3_six_world.sdf"
SCENE_PATH = PACKAGE / "config" / "scene_six.yaml"
MOTION_PATH = PACKAGE / "config" / "motion_points_six.yaml"
CLIENT_PATH = PACKAGE / "task3_six_sim" / "sort_all_client.py"


def _load_yaml(path):
    try:
        import yaml
    except ModuleNotFoundError:
        raise AssertionError("PyYAML is required for the six-object config test")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class SixSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ElementTree.parse(WORLD_PATH).getroot().find("world")
        cls.scene = _load_yaml(SCENE_PATH)
        cls.motion = _load_yaml(MOTION_PATH)

    def test_new_world_is_separate_and_has_six_grids(self):
        self.assertEqual(self.world.attrib["name"], "task3_six_world")
        names = {model.attrib["name"] for model in self.world.findall("model")}
        self.assertTrue({f"grid_p{i}" for i in range(1, 7)}.issubset(names))
        self.assertIn("yellow_region", names)
        self.assertIn("green_region", names)

    def test_world_has_three_yellow_and_three_green_dynamic_blocks(self):
        yellow = [self.world.find(f"model[@name='orange_battery_{i}']") for i in range(1, 4)]
        green = [self.world.find(f"model[@name='green_can_{i}']") for i in range(1, 4)]
        for model in yellow + green:
            self.assertIsNotNone(model)
            self.assertEqual(model.findtext("static"), "false")
            self.assertEqual(model.findtext(".//collision/geometry/box/size"), "0.040 0.018 0.050")

    def test_grid_and_object_centres_match_scene_configuration(self):
        object_at_grid = {
            "P1": "orange_battery_1",
            "P2": "green_can_1",
            "P3": "orange_battery_2",
            "P4": "green_can_2",
            "P5": "orange_battery_3",
            "P6": "green_can_3",
        }
        for grid_id, object_name in object_at_grid.items():
            expected = self.scene["grids"][grid_id]
            grid_pose = [
                float(value)
                for value in self.world.find(
                    f"model[@name='grid_{grid_id.lower()}']/pose"
                ).text.split()
            ]
            object_pose = [
                float(value)
                for value in self.world.find(
                    f"model[@name='{object_name}']/pose"
                ).text.split()
            ]
            self.assertEqual(grid_pose[:2], expected)
            self.assertEqual(object_pose[:2], expected)

    def test_scene_and_motion_define_six_pickups_and_three_slots_per_colour(self):
        self.assertEqual(set(self.scene["grids"]), {f"P{i}" for i in range(1, 7)})
        self.assertEqual(set(self.motion["picks"]), set(self.scene["grids"]))
        expected_bins = {
            *(f"YELLOW_{i}" for i in range(1, 4)),
            *(f"GREEN_{i}" for i in range(1, 4)),
        }
        self.assertEqual(set(self.scene["bins"]), expected_bins)
        self.assertEqual(set(self.motion["bins"]), expected_bins)
        self.assertEqual(len(self.scene["objects"]), 6)
        self.assertEqual(self.motion["gripper"]["closed_rad"], -0.65)

    def test_all_joint_poses_are_six_values_and_within_limits(self):
        lower = self.motion["joint_limits_deg"]["lower"]
        upper = self.motion["joint_limits_deg"]["upper"]
        poses = [self.motion["home"], self.motion["transfer_clearance"]]
        for phases in self.motion["picks"].values():
            poses.extend(phases["approach"])
            poses.append(phases["above"])
            poses.extend(phases["descent"])
            poses.append(phases["pick"])
            if "press" in phases:
                poses.append(phases["press"])
        for phases in self.motion["bins"].values():
            poses.extend((phases["above"], phases["place"]))
        for pose in poses:
            self.assertEqual(len(pose), 6)
            for value, lo, hi in zip(pose, lower, upper):
                self.assertGreaterEqual(value, lo)
                self.assertLessEqual(value, hi)

    def test_sort_all_client_has_exactly_three_jobs_per_colour(self):
        tree = ast.parse(CLIENT_PATH.read_text(encoding="utf-8"))
        assignment = next(
            node for node in tree.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "SORT_JOBS" for target in node.targets)
        )
        jobs = ast.literal_eval(assignment.value)
        self.assertEqual(len(jobs), 6)
        self.assertEqual(sum(bin_id.startswith("YELLOW_") for _, bin_id, _ in jobs), 3)
        self.assertEqual(sum(bin_id.startswith("GREEN_") for _, bin_id, _ in jobs), 3)


if __name__ == "__main__":
    unittest.main()
