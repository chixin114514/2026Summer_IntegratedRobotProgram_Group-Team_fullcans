"""Hold a fixed arm pose + gripper command so `ign model` can be queried."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm-prefix", required=True)
    ap.add_argument("--grip-prefix", required=True)
    ap.add_argument("--pose", required=True, help="six comma-separated degrees")
    ap.add_argument("--grip", type=float, required=True)
    ap.add_argument("--seconds", type=float, default=40.0)
    a = ap.parse_args()
    pose = [float(v) for v in a.pose.split(",")]
    rclpy.init()
    node = Node("task3_hold")
    arm_pubs = [node.create_publisher(Float64, f"{a.arm_prefix}/joint{i}/cmd_pos", 10) for i in range(1, 7)]
    grip_pubs = [node.create_publisher(Float64, f"{a.grip_prefix}/{n}_cmd_pos", 10)
                 for n in ("left3", "left2", "left1", "right3", "right2", "right1")]
    end = time.time() + a.seconds
    while time.time() < end:
        for p, v in zip(arm_pubs, pose):
            m = Float64(); m.data = math.radians(v); p.publish(m)
        for p, v in zip(grip_pubs, (a.grip, a.grip, -a.grip, -a.grip, -a.grip, a.grip)):
            m = Float64(); m.data = v; p.publish(m)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
