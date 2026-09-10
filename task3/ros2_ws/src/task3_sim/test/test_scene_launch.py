import ast
import re
import unittest
from pathlib import Path
from xml.etree import ElementTree


TASK3_ROOT = Path(__file__).resolve().parents[4]
LAUNCH_PATH = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim" / "launch" / "task3_scene.launch.py"
PACKAGE_PATH = TASK3_ROOT / "ros2_ws" / "src" / "task3_sim" / "package.xml"


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
        ):
            self.assertNotIn(forbidden, self.text)

    def test_launch_adapts_all_immutable_control_topics_to_task3_namespace(self):
        arm_pairs = [
            (f"/task2/joint{index}/cmd_pos", f"/task3/arm/joint{index}/cmd_pos")
            for index in range(1, 7)
        ]
        gripper_names = ("left3", "left2", "left1", "right3", "right2", "right1")
        gripper_pairs = [
            (
                f"/task2/gripper/{name}_cmd_pos",
                f"/task3/gripper/{name}_cmd_pos",
            )
            for name in gripper_names
        ]
        for gazebo_name, ros_name in arm_pairs + gripper_pairs:
            self.assertIn(gazebo_name, self.text)
            self.assertIn(ros_name, self.text)
        self.assertIn("remappings", self.text)
        self.assertIn("std_msgs/msg/Float64]ignition.msgs.Double", self.text)
        self.assertIn(
            "/world/task3_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            self.text,
        )

    def test_launch_starts_motion_server_with_task3_configuration_and_sim_time(self):
        self.assertIn('executable="pick_sort_server"', self.text)
        self.assertIn('"use_sim_time": True', self.text)
        self.assertIn("motion_points.yaml", self.text)
        self.assertIn("scene.yaml", self.text)
        self.assertIn('name="task3_bridge"', self.text)
        self.assertIn('name="pick_sort_server"', self.text)

    def test_task3_package_declares_motion_server_dependencies(self):
        package_root = ElementTree.parse(PACKAGE_PATH).getroot()
        declared = {
            element.text
            for element in package_root
            if element.tag in {"depend", "exec_depend"} and element.text
        }
        for dependency in ("rclpy", "std_msgs", "tf2_msgs", "task3_interfaces"):
            self.assertIn(dependency, declared)

    def test_bridge_constants_are_twelve_unique_one_to_one_adapters(self):
        assignments = {
            node.targets[0].id: ast.literal_eval(node.value)
            for node in self.tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id in {"CONTROL_REMAPS", "BRIDGE_ARGUMENTS"}
        }
        remaps = assignments["CONTROL_REMAPS"]
        bridges = assignments["BRIDGE_ARGUMENTS"]
        self.assertEqual(len(remaps), 12)
        self.assertEqual(len(set(remaps)), 12)
        for gazebo_topic, ros_topic in remaps:
            self.assertTrue(gazebo_topic.startswith("/task2/"))
            self.assertTrue(ros_topic.startswith("/task3/"))
            self.assertTrue(any(argument.startswith(gazebo_topic + "@") for argument in bridges))
            self.assertNotEqual(gazebo_topic, ros_topic)
        task2_bridges = [argument for argument in bridges if argument.startswith("/task2/")]
        self.assertEqual(len(task2_bridges), 13)
        command_bridges = [
            argument for argument in task2_bridges if "]ignition.msgs.Double" in argument
        ]
        state_bridges = [
            argument for argument in task2_bridges if "[ignition.msgs.Model" in argument
        ]
        self.assertEqual(len(command_bridges), 12)
        self.assertEqual(len(state_bridges), 1)
        self.assertTrue(state_bridges[0].startswith("/task2/gazebo/joint_state@"))

    def test_immutable_task2_topics_are_only_explicit_one_to_one_adapters(self):
        allowed_gazebo_topics = {
            *(f"/task2/joint{index}/cmd_pos" for index in range(1, 7)),
            *(
                f"/task2/gripper/{name}_cmd_pos"
                for name in ("left3", "left2", "left1", "right3", "right2", "right1")
            ),
            "/task2/gazebo/joint_state",
        }
        task2_topics = set(re.findall(r"/task2/[A-Za-z0-9_/]+", self.text))
        self.assertTrue(task2_topics)
        self.assertEqual(task2_topics, allowed_gazebo_topics)
        for gazebo_topic in allowed_gazebo_topics:
            self.assertIn(gazebo_topic, self.text)
        self.assertNotIn("task2_sim", self.text)
        self.assertNotIn("task2/", self.text.replace("/task2/", ""))


if __name__ == "__main__":
    unittest.main()
