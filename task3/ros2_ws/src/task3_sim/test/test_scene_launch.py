import ast
import unittest
from pathlib import Path


TASK3_ROOT = Path(__file__).resolve().parents[4]
LAUNCH_PATH = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim" / "launch" / "task3_scene.launch.py"


class SceneLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not LAUNCH_PATH.is_file():
            raise AssertionError(f"missing launch file: {LAUNCH_PATH}")
        cls.text = LAUNCH_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.text, filename=str(LAUNCH_PATH))

    def test_launch_starts_ignition_world_and_spawns_mecharm(self):
        self.assertIn('"ign"', self.text)
        self.assertIn('"gazebo"', self.text)
        self.assertIn('"-r"', self.text)
        self.assertIn("task3_world.sdf", self.text)
        self.assertIn("mecharm_270_gazebo.urdf", self.text)
        self.assertIn("/world/task3_world/create", self.text)
        self.assertIn("mecharm_270", self.text)
        self.assertIn("0.40", self.text)

    def test_launch_starts_robot_state_publisher_and_bridges_clock_and_image(self):
        self.assertIn("robot_state_publisher", self.text)
        self.assertIn("ros_ign_bridge", self.text)
        self.assertIn("parameter_bridge", self.text)
        self.assertIn("/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock", self.text)
        self.assertIn(
            "/task3/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            self.text,
        )

    def test_launch_has_no_task2_runtime_reference(self):
        for forbidden in (
            "task2_sim",
            "task_manager",
            "result_monitor",
            "task_points.yaml",
            "/task2/",
        ):
            self.assertNotIn(forbidden, self.text)


if __name__ == "__main__":
    unittest.main()
