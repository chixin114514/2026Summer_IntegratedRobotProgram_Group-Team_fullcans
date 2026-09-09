import ast
import unittest
from pathlib import Path


TASK3_ROOT = Path(__file__).resolve().parents[4]
SERVER_PATH = (
    TASK3_ROOT
    / "ros2_ws"
    / "src"
    / "task3_sim"
    / "task3_sim"
    / "pick_sort_server.py"
)


def _contains_attribute(node, attribute):
    return any(
        isinstance(child, ast.Attribute) and child.attr == attribute
        for child in ast.walk(node)
    )


class MotionServerContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = SERVER_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.text, filename=str(SERVER_PATH))
        cls.functions = {
            node.name: node
            for node in ast.walk(cls.tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }

    def test_server_has_no_fabricated_active_pose_sampler(self):
        self.assertNotIn("_record_active_sample", self.functions)
        self.assertNotIn("_record_active_sample", self.text)

    def test_failure_result_preserves_pick_verified_parameter(self):
        function = self.functions["_failure_result"]
        names = [argument.arg for argument in function.args.args]
        self.assertIn("pick_verified", names)
        self.assertIn("pick_verified", self.text)

    def test_final_cancel_gate_precedes_place_check(self):
        function = self.functions["_execute_callback"]
        try_block = next(node for node in function.body if isinstance(node, ast.Try))
        place_index = next(
            index
            for index, node in enumerate(try_block.body)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "place_verified"
                for target in node.targets
            )
        )
        cancel_indices = [
            index
            for index, node in enumerate(try_block.body)
            if isinstance(node, ast.If) and _contains_attribute(node.test, "is_cancel_requested")
        ]
        self.assertTrue(cancel_indices)
        self.assertTrue(any(index < place_index for index in cancel_indices))

    def test_safe_reset_uses_smooth_home_duration_and_keeps_gripper_open(self):
        function = self.functions["_safe_reset"]
        function_text = ast.get_source_segment(self.text, function)
        self.assertIn('durations_s', function_text)
        self.assertIn('_rate_hz', function_text)
        self.assertIn('_smoothstep', function_text)
        self.assertIn('_publish_gripper(self._open)', function_text)

    def test_motion_step_updates_last_command_each_tick(self):
        function = self.functions["_run_step"]
        loop = next(node for node in ast.walk(function) if isinstance(node, ast.For))
        loop_text = ast.get_source_segment(self.text, loop)
        self.assertIn("self._last_arm", loop_text)
        self.assertIn("self._last_gripper", loop_text)

    def test_settle_feedback_uses_constant_completed_release_progress(self):
        function = self.functions["_run_settle"]
        function_text = ast.get_source_segment(self.text, function)
        self.assertNotIn("tick / ticks", function_text)
        self.assertIn("progress = (index + 1.0) / total", function_text)

    def test_pose_callback_groups_frames_and_appends_once_per_object(self):
        function = self.functions["_pose_callback"]
        function_text = ast.get_source_segment(self.text, function)
        self.assertIn("frame_priority", function_text)
        self.assertIn("selected", function_text)
        self.assertIn("selected.items()", function_text)
        append_calls = [
            node
            for node in ast.walk(function)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "append"
        ]
        self.assertEqual(len(append_calls), 1)

    def test_placement_cancel_gate_is_after_check_and_before_succeed(self):
        function = self.functions["_execute_callback"]
        try_block = next(node for node in function.body if isinstance(node, ast.Try))
        place_index = next(
            index
            for index, node in enumerate(try_block.body)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == "place_verified"
                for target in node.targets
            )
        )
        succeed_index = next(
            index
            for index, node in enumerate(try_block.body)
            if isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Attribute)
            and node.value.func.attr == "succeed"
        )
        cancel_indices = [
            index
            for index, node in enumerate(try_block.body)
            if isinstance(node, ast.If) and _contains_attribute(node.test, "is_cancel_requested")
        ]
        self.assertTrue(any(place_index < index < succeed_index for index in cancel_indices))


if __name__ == "__main__":
    unittest.main()
