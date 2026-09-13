"""Report where every object ended up relative to the two collection regions."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

REGIONS = {"YELLOW": (0.131557, -0.047883), "GREEN": (-0.047883, -0.131557)}
HALF = 0.065
GROUPS = {
    "yellow": ("orange_battery_1", "orange_battery_2", "orange_battery_3"),
    "green": ("green_can_1", "green_can_2", "green_can_3"),
}


class Watcher(Node):
    def __init__(self):
        super().__init__("region_watch")
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.poses = {}
        self.create_subscription(TFMessage, "/task3_six/gazebo/pose/info", self.cb, qos)

    def cb(self, msg):
        for tr in msg.transforms:
            fid = tr.child_frame_id
            if "::" in fid:
                continue
            t = tr.transform.translation
            self.poses[fid] = (t.x, t.y, t.z)


def main():
    rclpy.init()
    node = Watcher()
    end = time.time() + 6.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"{'object':18s} {'x':>9s} {'y':>9s} {'z':>8s}   verdict")
    ok = {"yellow": 0, "green": 0}
    for group, names in GROUPS.items():
        want = REGIONS["YELLOW"] if group == "yellow" else REGIONS["GREEN"]
        for name in names:
            pose = node.poses.get(name)
            if pose is None:
                print(f"{name:18s} {'-':>9s} {'-':>9s} {'-':>8s}   NOT FOUND")
                continue
            inside = abs(pose[0] - want[0]) <= HALF and abs(pose[1] - want[1]) <= HALF
            if inside:
                ok[group] += 1
            print(f"{name:18s} {pose[0]:+9.5f} {pose[1]:+9.5f} {pose[2]:8.4f}   "
                  f"{'IN ' + group + ' region' if inside else 'OUTSIDE'}")
    print()
    print(f"yellow blocks in YELLOW region: {ok['yellow']}/3")
    print(f"green  blocks in GREEN  region: {ok['green']}/3")
    print("RESULT:", "PASS" if ok["yellow"] == 3 and ok["green"] == 3 else "FAIL")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
