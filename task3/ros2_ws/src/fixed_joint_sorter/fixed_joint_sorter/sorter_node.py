import threading
import time
from collections import deque
from collections import Counter

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .config import ConfigError, load_config, validate_config
from .detection import classify_initial_layout, parse_message
from .robot import JointRobot, RobotError, run_pick_place_cycle


class FixedSorter(Node):
    def __init__(self):
        super().__init__("fixed_joint_sort")
        self.declare_parameter("config", "")
        self.declare_parameter("live", False)
        self.cfg = load_config(self.get_parameter("config").value)
        self.live = bool(self.get_parameter("live").value)
        validate_config(self.cfg, live=self.live)
        self.frames = deque(maxlen=int(self.cfg["vision"]["history_frames"]))
        self.lock = threading.Lock()
        self._vision_subscription = self.create_subscription(String, self.cfg["vision"]["topic"], self._on_detection, 10)
        self.robot = None

    def _on_detection(self, message):
        try:
            frame = parse_message(message.data)
            with self.lock:
                self.frames.append(frame)
        except Exception as exc:
            self.get_logger().warning("忽略无效 YOLO 消息: {}".format(exc))

    def initial_classifications(self):
        with self.lock:
            frames = list(self.frames)
        return classify_initial_layout(
            frames,
            set(self.cfg["class_routes"]),
            float(self.cfg["vision"]["min_confidence"]),
            int(self.cfg["vision"]["required_votes"]),
        )

    def wait_for_initial_layout(self):
        timeout = float(self.cfg["vision"].get("classification_timeout_s", 0))
        deadline = None if timeout <= 0 else time.monotonic() + timeout
        while True:
            result = self.initial_classifications()
            if len(result) == 6:
                return result
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError("初始 YOLO 分类等待超时")
            time.sleep(0.1)

    def execute(self):
        classes = self.wait_for_initial_layout()
        routed_counts = Counter(self.cfg["class_routes"][class_name] for class_name in classes.values())
        for bin_name, bin_cfg in self.cfg["bins"].items():
            expected_count = bin_cfg["expected_count"]
            if routed_counts[bin_name] != expected_count:
                raise RuntimeError("初始分类数量异常：{} 有 {} 个物块，预期为 {} 个；请检查识别".format(bin_name, routed_counts[bin_name], expected_count))
        if not self.live:
            return
        self.destroy_subscription(self._vision_subscription)
        self.robot = JointRobot(self.cfg, self.get_logger().info)
        home = self.cfg["inspection_home_angles"]
        self.robot.move(home, "startup_home")
        for station in self.cfg["stations"]:
            prefix = station["name"]
            class_name = classes[prefix]
            bin_name = self.cfg["class_routes"][class_name]
            drop_point = self.cfg["bins"][bin_name]["drop_point"]
            run_pick_place_cycle(self.robot, station, drop_point, home, prefix)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = FixedSorter()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        node.execute()
    except KeyboardInterrupt:
        if node and node.robot:
            node.robot.stop()
    except (ConfigError, RobotError, RuntimeError, ValueError) as exc:
        if node:
            node.get_logger().error(str(exc))
        else:
            print("配置错误: {}".format(exc))
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        rclpy.shutdown()
