"""订阅 Task1 的 YOLO 检测结果并保留最新一帧。

话题：``/yolo/detections``，类型 ``std_msgs/msg/String``，内容是 JSON：

    {"fps": 8.7, "object_count": 2,
     "objects": [{"class_id": 0, "class_name": "charger", "confidence": 0.91,
                  "bbox": {"x1": 111, "y1": 222, "x2": 333, "y2": 401}}]}

注意 bbox 是**像素**，不是三维坐标；转成桌面坐标要靠
``task3_vision.homography``。
"""

from __future__ import annotations

import json
import threading
import time

from rclpy.node import Node
from std_msgs.msg import String


class DetectionListener(Node):
    """把最新一帧检测结果缓存在内存里，供主线程随时取用。"""

    def __init__(self, config):
        super().__init__("task3_vision_detector")
        camera = config["camera"]
        topic = str(camera.get("detections_topic", "/yolo/detections"))

        self._lock = threading.Lock()
        self._stamp = 0.0
        self._objects = []
        self._frames = 0
        self._seen_names = set()

        self.create_subscription(String, topic, self._callback, 10)
        self.get_logger().info(f"监听 {topic}（std_msgs/String, JSON）")

    # ------------------------------------------------------------ 解包
    def _callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return

        objects = []
        for item in payload.get("objects") or []:
            if not isinstance(item, dict):
                continue
            bbox = item.get("bbox") or {}
            try:
                x1 = float(bbox["x1"])
                y1 = float(bbox["y1"])
                x2 = float(bbox["x2"])
                y2 = float(bbox["y2"])
                confidence = float(item.get("confidence", 0.0))
                class_id = int(item.get("class_id", -1))
            except (KeyError, TypeError, ValueError):
                continue
            objects.append({
                "class_id": class_id,
                "class_name": str(item.get("class_name", "")),
                "confidence": confidence,
                # bbox 中心就是物体的像点
                "u": (x1 + x2) / 2.0,
                "v": (y1 + y2) / 2.0,
                "bbox": [x1, y1, x2, y2],
            })

        with self._lock:
            self._stamp = time.monotonic()
            self._objects = objects
            self._frames += 1
            self._seen_names.update(item["class_name"] for item in objects)

    # ------------------------------------------------------------ 读取
    def snapshot(self, max_age_s):
        """返回 ``(帧数, 时间戳, 目标列表)``。数据过期时目标列表为 None。"""
        with self._lock:
            stamp = self._stamp
            objects = [dict(item) for item in self._objects]
            frames = self._frames
        fresh = stamp > 0.0 and (time.monotonic() - stamp) <= float(max_age_s)
        return frames, stamp, (objects if fresh else None)

    def seen_names(self):
        with self._lock:
            return sorted(self._seen_names)
