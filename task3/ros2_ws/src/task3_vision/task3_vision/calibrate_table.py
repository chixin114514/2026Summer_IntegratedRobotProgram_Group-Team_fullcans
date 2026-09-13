"""桌面标定：把 YOLO 的像素坐标标成机械臂桌面坐标。

先在 Jetson 上把 Task1 的 YOLO 节点跑起来（它发布 /yolo/detections），
另开一个终端跑本节点：

    ros2 run task3_vision calibrate_table

桌面坐标系约定（必须先理解，否则标定出来的角度会整体转一个方向）：

    原点 = 机械臂底座中心
    +X   = J1 = 0 时手臂伸出的方向（把手臂转到 J1=0 看它朝哪，那就是 +X）
    +Y   = 从 +X 逆时针 90 度（坐在 +X 方向看，Y 在左边）
    +Z   = 向上
    单位 = 米

流程很简单：在桌面上选 4 个以上位置（放胶带十字做标记），用尺子量出每个点
相对底座的 X、Y。每个点把一个物块放上去，等检测框稳定，输入该点的 X Y。
程序把"像素 ↔ 桌面坐标"配对起来，解出单应矩阵，写到标定文件。

建议标定的 4 个点铺开一些（别都挤在一角），覆盖你实际要抓的整个区域，
重投影误差控制在 3~5 mm 以内。
"""

from __future__ import annotations

import os
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node

# 允许 `python3 calibrate_table.py` 直接跑（源码目录里没有安装好的包）
sys.path.insert(0, str(Path(__file__).resolve().parent))

from task3_vision import homography as homography_module          # noqa: E402
from task3_vision.detector import DetectionListener               # noqa: E402


def _load_config():
    path = (Path(__file__).resolve().parent / "config" / "vision_config.yaml")
    if path.exists():
        with path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}
    return {"camera": {"detections_topic": "/yolo/detections", "max_age_s": 2.0}}


class CalibrateNode(Node):
    def __init__(self):
        super().__init__("task3_vision_calibrate")
        self.declare_parameter("calibration_file", "~/.ros/task3_table_calibration.yaml")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("samples", 5)
        self.output = Path(os.path.expanduser(
            str(self.get_parameter("calibration_file").value)))
        self.image_size = (int(self.get_parameter("image_width").value),
                           int(self.get_parameter("image_height").value))
        self.samples = int(self.get_parameter("samples").value)
        self.detector = DetectionListener(_load_config())

    # ---------------------------------------------------------------
    def spin_for(self, seconds):
        """把订阅回调跑够这么多秒，保证后面的读数是新鲜的。"""
        deadline = time.monotonic() + float(seconds)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            rclpy.spin_once(self.detector, timeout_sec=0.05)

    def latest(self):
        self.spin_for(1.5)
        frames, stamp, objects = self.detector.snapshot(3.0)
        if objects is None:
            return None
        return objects


# ===================================================================
def _ask_xy(prompt):
    """让用户输入一个桌面坐标（米）。回车返回 None 表示结束标定。"""
    while True:
        raw = input(prompt).strip()
        if not raw:
            return None
        parts = raw.replace(",", " ").split()
        if len(parts) != 2:
            print("  请输入两个数，例如：0.150 -0.050")
            continue
        try:
            return float(parts[0]), float(parts[1])
        except ValueError:
            print("  解析失败，请重新输入")


def _choose(objects):
    """多个检测同时出现时让用户挑一个。返回选中的检测。"""
    ordered = sorted(objects, key=lambda item: -item["confidence"])
    if len(ordered) == 1:
        return ordered[0]
    print("  当前画面里有多个目标：")
    for index, item in enumerate(ordered):
        print(f"    [{index}] {item['class_name']:>16s}  "
              f"conf={item['confidence']:.2f}  "
              f"像素=({item['u']:.0f}, {item['v']:.0f})")
    raw = input("  输入要用的目标序号（回车=0）: ").strip()
    if not raw:
        return ordered[0]
    try:
        return ordered[int(raw)]
    except (ValueError, IndexError):
        print("  序号无效，用 0")
        return ordered[0]


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()

    print()
    print("=" * 62)
    print(" Task3 桌面标定（像素 -> 机械臂桌面坐标）")
    print("=" * 62)
    print(" 桌面坐标系：原点=底座中心, +X=J1=0 时手臂伸出方向, +Y=逆时针 90°, 单位米")
    print(f" 需要 {node.samples} 个点（>=4 即可，点多一些更准）")
    print(" 每个点：把一个物块放在标记上 -> 等检测稳定 -> 输入该点的 X Y")
    print(" 中途直接回车 = 结束并结算（已够 4 个点就出结果）")
    print()

    pairs = []
    while len(pairs) < node.samples:
        index = len(pairs) + 1
        print(f"--- 第 {index} 个点 ---")
        input("  把物块放到标记点上，放好后按回车读取检测...")
        objects = node.latest()
        if not objects:
            print("  没读到检测结果。确认 YOLO 节点在跑、话题名和 "
                  "camera.detections_topic 一致，然后重来这个点。")
            continue
        chosen = _choose(objects)
        print(f"  读到 {chosen['class_name']}  conf={chosen['confidence']:.2f}  "
              f"像素中心=({chosen['u']:.1f}, {chosen['v']:.1f})")
        xy = _ask_xy("  输入这个点的桌面 X Y（米）: ")
        if xy is None:
            break
        pairs.append(((chosen["u"], chosen["v"]), xy))
        print(f"  已记录 {len(pairs)} 组")

    if len(pairs) < 4:
        print(f"\n只有 {len(pairs)} 组点，至少需要 4 组，没有写入任何文件。")
        node.detector.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        return 1

    matrix = homography_module.solve_homography(pairs, node.image_size)
    if matrix is None:
        print("\n单应矩阵求解失败（点共线或退化）。换几个铺开的点重试。")
        node.detector.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        return 1

    errors = homography_module.reprojection_errors(matrix, pairs)
    print("\n标定结果（重投影误差越小越好）：")
    for position, (((u, v), (x, y)), error) in enumerate(zip(pairs, errors), start=1):
        print(f"  点{position}: 像素=({u:6.1f},{v:6.1f}) -> "
              f"桌面=({x:+.4f},{y:+.4f})  误差={error * 1000:6.2f} mm")
    worst = max(errors)
    print(f"  最大误差 = {worst * 1000:.2f} mm  " +
          ("（可接受）" if worst <= 0.005 else "（偏大，建议重新标定）"))

    payload = {
        "image_size": list(node.image_size),
        "homography": [[round(value, 12) for value in row] for row in matrix],
        "max_error_m": round(worst, 6),
        "created": datetime.now().isoformat(timespec="seconds"),
        "points": [
            {"pixel": [round(u, 2), round(v, 2)],
             "table_xy": [round(x, 6), round(y, 6)],
             "error_m": round(error, 6)}
            for ((u, v), (x, y)), error in zip(pairs, errors)
        ],
    }
    node.output.parent.mkdir(parents=True, exist_ok=True)
    with node.output.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)
    print(f"\n已写入 {node.output}")
    print("现在可以跑：ros2 run task3_vision vision_sort --ros-args -p dry_run:=true")

    node.detector.destroy_node()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
