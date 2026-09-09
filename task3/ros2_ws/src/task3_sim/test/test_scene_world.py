import math
import unittest
from pathlib import Path
from xml.etree import ElementTree

TASK3_ROOT = Path(__file__).resolve().parents[4]
WORLD_PATH = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim" / "worlds" / "task3_world.sdf"
SCENE_CONFIG = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim" / "config" / "scene.yaml"


def _floats(text):
    return [float(value) for value in text.split()]


def _near(actual, expected, tolerance=1e-4):
    return math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance)


def _load_yaml(path):
    """Use PyYAML when available, with a tiny stdlib fallback for local checks."""
    try:
        import yaml
    except ModuleNotFoundError:
        # The project target installs python3-yaml on ROS 2 hosts.  Keeping
        # this fallback lets the asset/scene tests run on a bare macOS Python.
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


class SceneWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not WORLD_PATH.is_file():
            raise AssertionError(f"missing SDF: {WORLD_PATH}")
        cls.root = ElementTree.parse(WORLD_PATH).getroot()
        cls.world = cls.root.find("world")
        if cls.world is None:
            raise AssertionError("SDF has no world element")
        if not SCENE_CONFIG.is_file():
            raise AssertionError(f"missing scene config: {SCENE_CONFIG}")
        cls.config = _load_yaml(SCENE_CONFIG)

    def test_world_name_and_required_models(self):
        self.assertEqual(self.root.attrib.get("version"), "1.8")
        self.assertEqual(self.world.attrib.get("name"), "task3_world")
        names = {model.attrib.get("name") for model in self.world.findall("model")}
        required = {
            "ground_plane",
            "table",
            "grid_p1",
            "grid_p2",
            "grid_p3",
            "grid_p4",
            "bin_a",
            "bin_b",
            "orange_battery_1",
            "green_can_1",
            "overhead_camera",
        }
        self.assertTrue(required.issubset(names), sorted(required - names))

    def test_required_world_systems_are_loaded(self):
        plugins = self.world.findall("plugin")
        plugin_names = {
            plugin.attrib.get("name", "")
            for plugin in plugins
        }
        plugin_text = "\n".join(ElementTree.tostring(plugin, encoding="unicode") for plugin in plugins)
        for system in ("Physics", "UserCommands", "SceneBroadcaster", "Sensors"):
            self.assertTrue(
                any(system in name for name in plugin_names) or system in plugin_text,
                f"missing Gazebo {system} system",
            )

    def test_world_has_directional_sun_light(self):
        sun = self.world.find("light[@name='sun']")
        self.assertIsNotNone(sun)
        self.assertEqual(sun.attrib.get("type"), "directional")
        direction = _floats(sun.findtext("direction"))
        self.assertEqual(len(direction), 3)
        self.assertTrue(any(value != 0.0 for value in direction))
        self.assertEqual(len(_floats(sun.findtext("diffuse"))), 4)
        self.assertEqual(len(_floats(sun.findtext("specular"))), 4)
        self.assertEqual(sun.findtext("cast_shadows"), "true")

    def test_camera_is_overhead_rgb_640x480_at_30hz(self):
        camera_model = self.world.find("model[@name='overhead_camera']")
        self.assertIsNotNone(camera_model)
        pose = _floats(camera_model.findtext("pose"))
        expected_pose = [0.0, 0.0, 1.35, 0.0, 1.5708, 0.0]
        for actual, expected in zip(pose, expected_pose):
            self.assertTrue(_near(actual, expected), (pose, expected_pose))

        sensor = camera_model.find(".//sensor[@type='camera']")
        self.assertIsNotNone(sensor)
        self.assertTrue(_near(float(sensor.findtext("update_rate")), 30.0))
        camera = sensor.find("camera")
        self.assertIsNotNone(camera)
        self.assertTrue(_near(float(camera.findtext("horizontal_fov")), 1.05, 0.02))
        self.assertEqual(camera.findtext("image/width"), "640")
        self.assertEqual(camera.findtext("image/height"), "480")
        self.assertEqual(camera.findtext("image/format"), "R8G8B8")
        self.assertTrue(_near(float(camera.findtext("clip/near")), 0.05))
        self.assertTrue(_near(float(camera.findtext("clip/far")), 5.0))
        self.assertEqual(sensor.findtext("topic"), "/task3/camera/image_raw")

    def test_table_and_fixed_grid_geometry(self):
        table = self.world.find("model[@name='table']")
        self.assertIsNotNone(table)
        table_box = table.find(".//collision/geometry/box/size")
        self.assertEqual(_floats(table_box.text), [0.80, 0.70, 0.40])
        table_pose = _floats(table.findtext("pose"))
        self.assertTrue(_near(table_pose[2], 0.20))

        expected = {
            "grid_p1": (0.13213, 0.06812),
            "grid_p2": (0.08037, -0.12506),
            "grid_p3": (-0.08037, 0.12506),
            "grid_p4": (-0.13213, -0.06812),
        }
        for name, (x, y) in expected.items():
            grid = self.world.find(f"model[@name='{name}']")
            self.assertIsNotNone(grid)
            pose = _floats(grid.findtext("pose"))
            self.assertTrue(_near(pose[0], x) and _near(pose[1], y), (name, pose))
            size = _floats(grid.find(".//visual/geometry/box/size").text)
            self.assertTrue(_near(size[0], 0.09) and _near(size[1], 0.09), (name, size))
            self.assertLess(size[2], 0.01, (name, size))

    def test_targets_are_dynamic_with_inertial_collision_and_required_shapes(self):
        orange = self.world.find("model[@name='orange_battery_1']")
        green = self.world.find("model[@name='green_can_1']")
        self.assertIsNotNone(orange)
        self.assertIsNotNone(green)
        for model in (orange, green):
            self.assertEqual(model.findtext("static"), "false")
            inertial = model.find(".//link/inertial")
            self.assertIsNotNone(inertial)
            self.assertGreater(float(inertial.findtext("mass")), 0.0)
            for axis in ("ixx", "iyy", "izz"):
                self.assertGreater(float(inertial.findtext(f"inertia/{axis}")), 0.0)
            self.assertIsNotNone(model.find(".//link/collision"))
            self.assertIsNotNone(model.find(".//link/collision/surface/friction"))

        orange_size = _floats(orange.find(".//collision/geometry/box/size").text)
        self.assertEqual(orange_size, [0.070, 0.035, 0.035])
        green_cylinder = green.find(".//collision/geometry/cylinder")
        self.assertEqual(float(green_cylinder.findtext("radius")), 0.0175)
        self.assertEqual(float(green_cylinder.findtext("length")), 0.070)
        green_pose = _floats(green.findtext("pose"))
        self.assertTrue(_near(green_pose[3], 0.0) and _near(green_pose[4], 1.5708), green_pose)

    def test_grids_are_visual_only_and_targets_start_above_table(self):
        table = self.world.find("model[@name='table']")
        table_pose = _floats(table.findtext("pose"))
        table_size = _floats(table.find(".//collision/geometry/box/size").text)
        table_top = table_pose[2] + table_size[2] / 2.0

        for name in ("grid_p1", "grid_p2", "grid_p3", "grid_p4"):
            grid = self.world.find(f"model[@name='{name}']")
            self.assertIsNotNone(grid)
            self.assertIsNone(grid.find(".//collision"), f"{name} must not collide")
            self.assertIsNotNone(grid.find(".//visual"), f"{name} must remain visible")

        orange = self.world.find("model[@name='orange_battery_1']")
        green = self.world.find("model[@name='green_can_1']")
        orange_pose = _floats(orange.findtext("pose"))
        orange_size = _floats(orange.find(".//collision/geometry/box/size").text)
        orange_lowest = orange_pose[2] - orange_size[2] / 2.0

        green_pose = _floats(green.findtext("pose"))
        green_radius = float(green.find(".//collision/geometry/cylinder/radius").text)
        green_lowest = green_pose[2] - green_radius

        # Grid markers have no collision, so the physical support is the table.
        # Keep a 0.5 mm clearance to avoid initial object/table penetration.
        support_clearance = 0.0005
        self.assertGreaterEqual(orange_lowest, table_top + support_clearance - 1e-6)
        self.assertGreaterEqual(green_lowest, table_top + support_clearance - 1e-6)

    def test_task3_scene_coordinates_are_present_in_yaml(self):
        self.assertNotIn("positions", self.config)
        self.assertEqual(self.config["table"]["top_z"], 0.4)
        self.assertEqual(self.config["arm"], [0.0, 0.0])
        self.assertEqual(self.config["grids"], {
            "P1": [0.13213, 0.06812],
            "P2": [0.08037, -0.12506],
            "P3": [-0.08037, 0.12506],
            "P4": [-0.13213, -0.06812],
        })
        self.assertEqual(self.config["bins"], {
            "BIN_A": [0.12557, -0.03365],
            "BIN_B": [-0.11782, 0.05494],
        })
        self.assertEqual(self.config["camera"]["topic"], "/task3/camera/image_raw")

    def test_yaml_grid_and_bin_coordinates_match_sdf(self):
        for grid_name, coordinates in self.config["grids"].items():
            model = self.world.find(f"model[@name='grid_{grid_name.lower()}']")
            self.assertIsNotNone(model, grid_name)
            pose = _floats(model.findtext("pose"))
            self.assertTrue(
                _near(pose[0], coordinates[0]) and _near(pose[1], coordinates[1]),
                (grid_name, coordinates, pose),
            )
        for bin_name, coordinates in self.config["bins"].items():
            model = self.world.find(f"model[@name='{bin_name.lower()}']")
            self.assertIsNotNone(model, bin_name)
            pose = _floats(model.findtext("pose"))
            self.assertTrue(
                _near(pose[0], coordinates[0]) and _near(pose[1], coordinates[1]),
                (bin_name, coordinates, pose),
            )

    def test_scene_contains_no_pose_teleport_or_task2_reference(self):
        text = WORLD_PATH.read_text(encoding="utf-8")
        for forbidden in ("SetEntityPose", "set_pose", "task2_sim", "/task2/"):
            self.assertNotIn(forbidden, text)


if __name__ == "__main__":
    unittest.main()
