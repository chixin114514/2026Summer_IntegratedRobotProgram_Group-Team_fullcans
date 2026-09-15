"""Classify the six fixed pickup grids from RGB images, then sort by colour."""

from __future__ import annotations

import sys
import time
from collections import defaultdict, deque
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from task3_interfaces.action import SortObject

from .color_detection import (
    build_sort_jobs,
    classify_roi,
    project_grids,
    stable_colours,
)


ACTION_NAME = "/task3_six/sort_object"


class VisionSortClient(Node):
    def __init__(self) -> None:
        super().__init__("task3_six_vision_sort")
        self.declare_parameter("scene_config", "")
        self.declare_parameter("camera_topic", "/task3_six/camera/image_raw")
        self.declare_parameter("scan_timeout_s", 12.0)
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("roi_half_size_px", 16)
        self.declare_parameter("min_coloured_pixels", 24)
        self.declare_parameter("min_confidence", 0.72)
        self.declare_parameter("object_top_z", 0.450)
        self.declare_parameter("pixel_offset_u", 0.0)
        self.declare_parameter("pixel_offset_v", 0.0)

        configured_scene = str(self.get_parameter("scene_config").value).strip()
        scene_path = (
            Path(configured_scene)
            if configured_scene
            else Path(get_package_share_directory("task3_six_sim"))
            / "config" / "scene_six.yaml"
        )
        if not scene_path.is_file():
            raise FileNotFoundError(f"scene config not found: {scene_path}")
        with scene_path.open("r", encoding="utf-8") as stream:
            self._scene = yaml.safe_load(stream) or {}
        camera = self._scene.get("camera", {}) or {}
        pose = camera.get("pose", [0.0, 0.0, 1.35, 0.0, 1.5708, 0.0])
        self._grid_pixels = project_grids(
            self._scene.get("grids", {}),
            camera.get("image_size", [640, 480]),
            float(camera.get("horizontal_fov", 1.05)),
            float(pose[2]),
            float(self.get_parameter("object_top_z").value),
            (
                float(self.get_parameter("pixel_offset_u").value),
                float(self.get_parameter("pixel_offset_v").value),
            ),
        )
        if len(self._grid_pixels) != 6:
            raise ValueError("vision sorter requires exactly six configured grids")

        self._confirm_frames = int(self.get_parameter("confirm_frames").value)
        self._histories = defaultdict(
            lambda: deque(maxlen=max(1, self._confirm_frames))
        )
        self._colours = None
        self._frame_count = 0
        self._unsupported_encoding = ""
        self._last_summary = ""
        topic = str(self.get_parameter("camera_topic").value)
        self.create_subscription(
            Image, topic, self._image_callback, qos_profile_sensor_data
        )
        self._client = ActionClient(self, SortObject, ACTION_NAME)
        self._last_stage = ""
        centres = ", ".join(
            f"{grid}=({uv[0]:.1f},{uv[1]:.1f})"
            for grid, uv in self._grid_pixels.items()
        )
        self.get_logger().info(f"视觉分拣监听 {topic}; ROI: {centres}")

    def _image_callback(self, message: Image) -> None:
        if self._colours is not None:
            return
        self._frame_count += 1
        summary = []
        pixels = bytes(message.data)
        half_size = int(self.get_parameter("roi_half_size_px").value)
        min_pixels = int(self.get_parameter("min_coloured_pixels").value)
        min_confidence = float(self.get_parameter("min_confidence").value)
        try:
            for grid_id, centre in self._grid_pixels.items():
                decision = classify_roi(
                    pixels, message.width, message.height,
                    message.step, message.encoding, centre,
                    half_size=half_size,
                    min_coloured_pixels=min_pixels,
                    min_confidence=min_confidence,
                )
                self._histories[grid_id].append(decision.colour)
                label = decision.colour or "?"
                summary.append(
                    f"{grid_id}={label}(Y{decision.yellow_pixels}/G{decision.green_pixels})"
                )
        except ValueError as exc:
            message_text = str(exc)
            if message_text != self._unsupported_encoding:
                self._unsupported_encoding = message_text
                self.get_logger().error(message_text)
            return
        text = " ".join(summary)
        if text != self._last_summary:
            self._last_summary = text
            self.get_logger().info("视觉识别: " + text)
        self._colours = stable_colours(self._histories, self._confirm_frames)
        if self._colours is not None:
            result = ", ".join(
                f"{grid}={colour}" for grid, colour in self._colours.items()
            )
            self.get_logger().info("视觉识别稳定: " + result)

    def _feedback(self, message) -> None:
        stage = str(message.feedback.stage)
        if stage != self._last_stage:
            self._last_stage = stage
            self.get_logger().info("stage: " + stage)

    def _scan(self) -> bool:
        deadline = time.monotonic() + float(
            self.get_parameter("scan_timeout_s").value
        )
        while rclpy.ok() and self._colours is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._colours is None:
            self.get_logger().error(
                f"视觉识别超时（收到 {self._frame_count} 帧）；不会移动机械臂"
            )
            return False
        return True

    def run(self) -> bool:
        if not self._scan():
            return False
        try:
            jobs = build_sort_jobs(self._colours)
        except ValueError as exc:
            self.get_logger().error(f"识别结果不安全: {exc}; 不会移动机械臂")
            return False
        plan = ", ".join(f"{grid}->{slot}" for grid, slot, _ in jobs)
        self.get_logger().info("视觉生成分拣计划: " + plan)

        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("分拣 Action server 不可用")
            return False
        failed = []
        for index, (grid_id, bin_id, colour) in enumerate(jobs, 1):
            self._last_stage = ""
            goal = SortObject.Goal()
            goal.grid_id = grid_id
            goal.bin_id = bin_id
            self.get_logger().info(
                f"[{index}/6] 相机判定 {grid_id} 为 {colour}，放到 {bin_id}"
            )
            sent = self._client.send_goal_async(goal, feedback_callback=self._feedback)
            rclpy.spin_until_future_complete(self, sent)
            handle = sent.result()
            if handle is None or not handle.accepted:
                self.get_logger().error(f"目标被拒绝: {grid_id}->{bin_id}")
                return False
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            response = result_future.result()
            result = None if response is None else response.result
            if result is None or not result.success:
                detail = "无结果" if result is None else result.message
                failed.append(f"{grid_id}->{bin_id}: {detail}")
                self.get_logger().error("分拣失败，继续下一块: " + failed[-1])
        if failed:
            self.get_logger().error("视觉分拣结束，但有失败: " + "; ".join(failed))
            return False
        self.get_logger().info("视觉分拣完成：3 个黄色和 3 个绿色物块均已送往对应区域")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionSortClient()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()
