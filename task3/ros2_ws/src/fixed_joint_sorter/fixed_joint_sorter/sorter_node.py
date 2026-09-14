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
from .robot import JointRobot, RobotError


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
        previous = None
        last_log = 0.0
        while True:
            result = self.initial_classifications()
            if result != previous:
                self.get_logger().info("初始分类进度 {}/6: {}".format(len(result), result))
                previous = result
            if len(result) == 6:
                return result
            if time.monotonic() - last_log >= 1.0:
                self.get_logger().info("保持6个物块和摄像头不动，等待初始分类稳定...")
                last_log = time.monotonic()
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError("初始 YOLO 分类等待超时")
            time.sleep(0.1)

    def execute(self):
        classes = self.wait_for_initial_layout()
        self.get_logger().info("========== 初始判断完成 ==========")
        for station in self.cfg["stations"]:
            class_name = classes[station["name"]]
            bin_name = self.cfg["class_routes"][class_name]
            label = self.cfg["bins"][bin_name]["label"]
            self.get_logger().info("{} = {} -> {}".format(station["name"], class_name, label))
        routed_counts = Counter(self.cfg["class_routes"][class_name] for class_name in classes.values())
        for bin_name, bin_cfg in self.cfg["bins"].items():
            if routed_counts[bin_name] != len(bin_cfg["slots"]):
                raise RuntimeError("初始分类数量异常：{} 有 {} 个物块，但只有 {} 个槽；请检查识别".format(bin_name, routed_counts[bin_name], len(bin_cfg["slots"])))
        self.get_logger().info("分类结果已锁定。后续不再读取摄像头，现在开始机械臂流程；摄像头可以关闭。")
        if not self.live:
            self.get_logger().info("DRY RUN 完成：没有连接或移动机械臂")
            return
        self.destroy_subscription(self._vision_subscription)
        self.robot = JointRobot(self.cfg, self.get_logger().info)
        used_slots = {name: 0 for name in self.cfg["bins"]}
        home = self.cfg["inspection_home_angles"]
        self.robot.move(home, "startup_home")
        for station in self.cfg["stations"]:
            prefix = station["name"]
            class_name = classes[prefix]
            bin_name = self.cfg["class_routes"][class_name]
            slot_index = used_slots[bin_name]
            if slot_index >= len(self.cfg["bins"][bin_name]["slots"]):
                raise RuntimeError("初始分类显示 {} 超过3个，{} 没有足够空位".format(class_name, bin_name))
            slot = self.cfg["bins"][bin_name]["slots"][slot_index]
            self.robot.gripper(close=False)
            self.robot.move(station["approach_angles"], prefix + "_approach")
            self.robot.move(station["pick_angles"], prefix + "_pick")
            self.robot.gripper(close=True)
            self.robot.move(station["retreat_angles"], prefix + "_retreat")
            self.get_logger().info("{}: {} -> {} 的第{}槽".format(prefix, class_name, bin_name, slot_index + 1))
            self.robot.move(home, prefix + "_transfer_home")
            self.robot.move(slot["approach_angles"], bin_name + "_approach_{}".format(slot_index + 1))
            self.robot.move(slot["place_angles"], bin_name + "_place_{}".format(slot_index + 1))
            self.robot.gripper(close=False)
            self.robot.move(slot["retreat_angles"], bin_name + "_retreat_{}".format(slot_index + 1))
            self.robot.move(home, prefix + "_return_home")
            used_slots[bin_name] += 1
        self.robot.move(home, "finish_home")
        self.get_logger().info("6 个物块分拣完成")


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
        if node:
            node.get_logger().info("用户按 Ctrl+C，停止")
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
