import unittest
from pathlib import Path
from xml.etree import ElementTree


TASK3_ROOT = Path(__file__).resolve().parents[4]
INTERFACE_ROOT = TASK3_ROOT / "ros2_ws" / "src" / "task3_interfaces"
ACTION_PATH = INTERFACE_ROOT / "action" / "SortObject.action"
PACKAGE_PATH = INTERFACE_ROOT / "package.xml"
CMAKE_PATH = INTERFACE_ROOT / "CMakeLists.txt"


class ActionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        for path in (ACTION_PATH, PACKAGE_PATH, CMAKE_PATH):
            if not path.is_file():
                raise AssertionError(f"missing interface file: {path}")
        cls.action_lines = [
            line.strip()
            for line in ACTION_PATH.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]
        cls.package_text = PACKAGE_PATH.read_text(encoding="utf-8")
        cls.cmake_text = CMAKE_PATH.read_text(encoding="utf-8")
        cls.package_root = ElementTree.parse(PACKAGE_PATH).getroot()

    def test_action_has_exact_goal_result_and_feedback_fields(self):
        self.assertEqual(
            self.action_lines,
            [
                "string grid_id",
                "string bin_id",
                "---",
                "bool success",
                "bool pick_verified",
                "bool place_verified",
                "string object_name",
                "string message",
                "---",
                "string stage",
                "float32 progress",
            ],
        )

    def test_interface_package_declares_minimal_action_generation_dependencies(self):
        for dependency in (
            "rosidl_default_generators",
            "action_msgs",
            "rosidl_default_runtime",
        ):
            self.assertIn(dependency, self.package_text)
        self.assertIn("rosidl_generate_interfaces", self.cmake_text)
        self.assertIn('"action/SortObject.action"', self.cmake_text)
        self.assertEqual(self.package_root.findtext("name"), "task3_interfaces")
        self.assertIn("ament_cmake", self.package_text)


if __name__ == "__main__":
    unittest.main()
