"""不用 ROS 的自检工具：检查配置里的坐标点能不能解出来、会不会超限位。

J5 被顶到限位（+-115°）通常不是算法问题，而是喂进去的目标点跑到机械臂够不到
的地方了——标定有偏差、物体落在画面边缘、或者落料槽位坐标填到了工作区外面。
这个工具把配置里的每个槽位、以及一整片工作区网格都验一遍，直接告诉你是哪个点
出问题。

用法（不需要跑 ROS，也不需要机械臂上电）：

    ros2 run task3_vision check_workspace
    ros2 run task3_vision check_workspace --config /path/to/vision_config.yaml
    python3 check_workspace.py --config /path/to/vision_config.yaml
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

try:
    from . import ik                      # ros2 run / colcon 安装后
except ImportError:                       # 直接 python3 check_workspace.py
    _HERE = Path(__file__).resolve().parent
    sys.path.insert(0, str(_HERE))
    # 同工作空间里 task3_six_sim 的正运动学也要能找到
    sys.path.insert(0, str(_HERE.parents[1] / "task3_six_sim"))
    import ik                             # noqa: E402

NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


def load_config(path=None):
    if path:
        return Path(path), yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    # 优先读安装目录，其次读源码目录（这样不装也能跑）
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory("task3_vision"))
                          / "config" / "vision_config.yaml")
    except Exception:                                  # noqa: BLE001
        pass
    candidates.append(Path(__file__).resolve().parents[1]
                      / "config" / "vision_config.yaml")
    for candidate in candidates:
        if candidate.exists():
            return candidate, yaml.safe_load(candidate.read_text(encoding="utf-8"))
    raise SystemExit("找不到 vision_config.yaml，请用 --config 指定")


class Workspace:
    def __init__(self, config):
        geometry = config["geometry"]
        limits = config["limits"]
        self.j4 = float(geometry["fixed_j4_deg"])
        self.grip = float(geometry["ik_grip_rad"])
        self.base_z = float(geometry["base_z_m"])
        self.seed = [float(v) for v in geometry["ik_seed_q"]]
        posture = geometry.get("ik_posture_q")
        self.posture = [float(v) for v in posture] if posture else None
        self.pick_z = float(geometry["pick_pad_z_m"])
        self.place_z = float(geometry["place_pad_z_m"])
        self.above = float(geometry["above_offset_m"])
        self.transfer = float(geometry["transfer_offset_m"])
        self.lower = [float(v) for v in limits["lower"]]
        self.upper = [float(v) for v in limits["upper"]]
        self.limits3 = ([self.lower[1], self.lower[2], self.lower[4]],
                        [self.upper[1], self.upper[2], self.upper[4]])

    def test(self, x, y, z):
        """返回 (是否可用, 方位角, 六个关节角, 残差, 超限的关节名)。"""
        bearing = math.degrees(math.atan2(y, x))
        q, residual = ik.solve((x, y, z), self.seed, bearing,
                               j4_deg=self.j4, grip_rad=self.grip,
                               base_z=self.base_z, posture=self.posture,
                               limits=self.limits3)
        arm = [bearing, q[0], q[1], self.j4, q[2], bearing]
        over = [NAMES[i] for i in range(6)
                if not (self.lower[i] - 1e-9 <= arm[i] <= self.upper[i] + 1e-9)]
        ok = residual <= ik.CONVERGED_M and not over
        return ok, bearing, arm, residual, over

    @property
    def heights(self):
        return [("pick", self.pick_z),
                ("pre_pick", self.pick_z + self.above * 0.3),
                ("place", self.place_z),
                ("above", self.pick_z + self.above),
                ("transfer", self.pick_z + self.transfer)]


def main(args=None):
    parser = argparse.ArgumentParser(description="检查工作区可达性")
    parser.add_argument("--config", default=None, help="vision_config.yaml 路径")
    options = parser.parse_args(args)

    path, config = load_config(options.config)
    space = Workspace(config)
    print(f"配置: {path}")
    print(f"抓取高度 {space.pick_z*1000:.1f}mm  放料高度 {space.place_z*1000:.1f}mm  "
          f"above +{space.above*1000:.0f}mm  转运 +{space.transfer*1000:.0f}mm")
    print(f"限位 J2[{space.lower[1]:.0f},{space.upper[1]:.0f}] "
          f"J3[{space.lower[2]:.0f},{space.upper[2]:.0f}] "
          f"J5[{space.lower[4]:.0f},{space.upper[4]:.0f}]")
    print()

    # ---------------------------------------------------------------
    print("=" * 88)
    print("1) 工作区网格：半径 x 高度   (O=可用  X=够不到/超限)")
    print("=" * 88)
    radii = [round(0.06 + 0.02 * i, 3) for i in range(10)]        # 0.06 .. 0.24
    header = "  z\\r   " + "".join(f"{r:>7.2f}" for r in radii)
    print(header)
    bad_cells = []
    for label, z in space.heights:
        row = f"  {label:<7s}"
        for radius in radii:
            # 方位角取最坏情况：整个圈扫一遍
            ok_any = False
            for bearing in range(0, 360, 15):
                x = radius * math.cos(math.radians(bearing))
                y = radius * math.sin(math.radians(bearing))
                ok, _b, _arm, _res, _over = space.test(x, y, z)
                if ok:
                    ok_any = True
                    break
            row += f"{'O' if ok_any else 'X':>7s}"
            if not ok_any:
                bad_cells.append((label, radius, z))
        print(row)
    print(f"  单位: 半径/高度 = 米；z 从上到下 = {[h[0] for h in space.heights]}")
    if bad_cells:
        print(f"  够不到的格子: "
              + ", ".join(f"{label}@r={radius}" for label, radius, _z in bad_cells))
    else:
        print("  所有格子都可用")

    # ---------------------------------------------------------------
    print()
    print("=" * 88)
    print("2) 配置里的落料槽位逐点检查")
    print("=" * 88)
    failures = 0
    for region_name, region in (config.get("regions") or {}).items():
        centre = region.get("centre")
        if centre:
            radius = math.hypot(float(centre[0]), float(centre[1]))
            print(f"  [{region_name}] centre=({centre[0]:+.3f},{centre[1]:+.3f}) "
                  f"r={radius:.3f} 差集半径={region.get('exclude_radius_m')}")
        for index, slot in enumerate(region.get("slots") or [], start=1):
            x, y = float(slot[0]), float(slot[1])
            radius = math.hypot(x, y)
            for label, z in (("place", space.place_z),
                             ("above", space.place_z + space.above)):
                ok, bearing, arm, residual, over = space.test(x, y, z)
                if not ok:
                    failures += 1
                print(f"    {region_name} 槽位{index} ({x:+.3f},{y:+.3f}) r={radius:.3f} "
                      f"{label:5s} J1={bearing:+7.1f} 残差={residual*1000:7.2f}mm "
                      f"{'OK ' if ok else 'FAIL'} "
                      + ("超限:" + ",".join(over) if over else "")
                      + ("" if ok else f"  关节="
                         + " ".join(f"{NAMES[i]}={arm[i]:+.1f}" for i in range(6))))
        # 区域中心也顺带验一下
        if centre:
            cx, cy = float(centre[0]), float(centre[1])
            ok, bearing, arm, residual, over = space.test(cx, cy, space.place_z)
            print(f"    {region_name} centre  {'' if ok else '不可用 ->'} "
                  f"残差={residual*1000:.2f}mm"
                  + (f" 超限:{','.join(over)}" if over else ""))

    print()
    print("=" * 88)
    if failures:
        print(f"有 {failures} 个槽位/高度解不出来。"
              f"把它们的坐标往底座方向挪一点，或者降低该区域的高度。")
    else:
        print("配置里所有槽位在放料和 above 两个高度都可用。")
    print("=" * 88)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
