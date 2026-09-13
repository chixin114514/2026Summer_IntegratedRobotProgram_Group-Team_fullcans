"""MechArm 270 真机通信：一次性下发目标 + 等测量到位。

和仿真里 20 Hz 流式插值下发完全不同：真机一次只发一个完整目标
（``send_angles``），然后轮询 ``get_angles()`` 直到六个关节都进入容差并
稳定，才走下一步。这样不会把舵机队列灌满，也不需要给每一步写死时长
（10%~30% 速度下每步要几秒，写死时长必然不是快了就是慢了）。

按用户要求：这里没有任何"失败就中止"的逻辑。读不到角度、等超时、发送
异常，一律只记日志然后继续往下走。
"""

from __future__ import annotations

import math
import time


class RealArm:
    """MechArm 270 的薄封装（socket 网口 或 serial 串口）。"""

    def __init__(self, config, logger=None):
        self.config = config
        self.robot = None
        self._logger = logger

    # ------------------------------------------------------------ 日志
    def _info(self, message):
        if self._logger is not None:
            self._logger.info(str(message))
        else:
            print(message, flush=True)

    def _warn(self, message):
        if self._logger is not None:
            self._logger.warn(str(message))
        else:
            print("WARN " + str(message), flush=True)

    # ------------------------------------------------------------ 连接
    def connect(self):
        """打开通信。只有这一步失败会抛异常——连都连不上就没法做任何事。"""
        robot_conf = self.config["robot"]
        driver = str(robot_conf.get("driver_type", "socket")).lower()

        if driver == "socket":
            try:
                from pymycobot import MechArmSocket
            except ImportError:
                from pymycobot.mecharmsocket import MechArmSocket
            ip = str(robot_conf.get("robot_ip", "192.168.31.247"))
            port = int(robot_conf.get("robot_port", 9000))
            self._info(f"连接机械臂 socket {ip}:{port}")
            self.robot = MechArmSocket(ip, port)
        else:
            try:
                from pymycobot.mecharm270 import MechArm270
            except ImportError:
                from pymycobot import MechArm270
            device = str(robot_conf.get("serial_device", "/dev/ttyUSB0"))
            baudrate = int(robot_conf.get("baudrate", 115200))
            self._info(f"连接机械臂串口 {device} @ {baudrate}")
            self.robot = MechArm270(device, baudrate)

        # fresh_mode = 0：一条一条执行，不把多个目标堆在队列里互相覆盖。
        setter = getattr(self.robot, "set_fresh_mode", None)
        if callable(setter):
            try:
                setter(0)
                self._info("fresh_mode = 0（排队执行，不累积）")
            except Exception as error:            # noqa: BLE001 - 不中止
                self._warn(f"设置 fresh_mode 失败（继续）: {error}")

        self._info(f"机械臂速度 = {robot_conf.get('arm_speed_percent')}%")
        return self

    # ------------------------------------------------------------ 反馈
    def measured_deg(self):
        """读一次六个关节角（度）。读不到返回 None，绝不抛异常。"""
        try:
            raw = self.robot.get_angles()
        except Exception as error:                # noqa: BLE001 - 不中止
            self._warn(f"get_angles 异常（继续）: {error}")
            return None
        if not isinstance(raw, (list, tuple)) or len(raw) != 6:
            return None
        try:
            values = [float(value) for value in raw]
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    # ------------------------------------------------------------ 关节
    def send_arm(self, arm_deg, speed=None):
        """下发六个关节角（度）。返回是否发送成功（失败也继续）。"""
        robot_conf = self.config["robot"]
        percent = int(robot_conf.get("arm_speed_percent", 20)
                      if speed is None else speed)
        values = [float(value) for value in arm_deg]
        try:
            result = self.robot.send_angles(values, percent)
            self._info("TX ["
                       + ", ".join(f"{value:8.2f}" for value in values)
                       + f"] speed={percent}% -> {result!r}")
            return True
        except Exception as error:                # noqa: BLE001 - 不中止
            self._warn(f"send_angles 异常（继续）: {error}")
            return False

    def move(self, arm_deg, label=""):
        """下发目标并等待到位。永远返回，不会因为超时中断流程。"""
        robot_conf = self.config["robot"]
        target = [float(value) for value in arm_deg]
        tolerance = float(robot_conf.get("arrival_tolerance_deg", 3.0))
        stable_s = float(robot_conf.get("arrival_stable_s", 0.4))
        timeout_s = float(robot_conf.get("arrival_timeout_s", 25.0))
        poll_s = float(robot_conf.get("poll_period_s", 0.2))

        self.send_arm(target)

        started = time.monotonic()
        stable_since = None
        last = None
        while time.monotonic() - started < timeout_s:
            now = time.monotonic()
            measured = self.measured_deg()
            if measured is not None:
                last = measured
                worst = max(abs(t - m) for t, m in zip(target, measured))
                if worst <= tolerance:
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= stable_s:
                        self._info(f"{label} 到位（最大残差 {worst:.2f}°，"
                                   f"耗时 {now - started:.1f}s）")
                        return True
                else:
                    stable_since = None
            time.sleep(poll_s)

        if last is not None:
            worst = max(abs(t - m) for t, m in zip(target, last))
            self._warn(f"{label} 等待超时（{timeout_s:.0f}s，残留 {worst:.2f}°）"
                       f"—— 不中止，继续下一步")
            self._info("    实测 ["
                       + ", ".join(f"{value:8.2f}" for value in last) + "]")
        else:
            self._warn(f"{label} 等待超时且读不到关节角 —— 不中止，继续下一步")
        return False

    # ------------------------------------------------------------ 夹爪
    def gripper(self, value, label=""):
        """设置夹爪开合。pymycobot 约定：100 = 全开，0 = 夹到底。"""
        robot_conf = self.config["robot"]
        value = int(max(0, min(100, int(value))))
        speed = int(robot_conf.get("gripper_speed_percent", 100))
        gripper_type = int(robot_conf.get("gripper_type", 1))
        try:
            try:
                result = self.robot.set_gripper_value(value, speed, gripper_type)
            except TypeError:
                # 老版本 pymycobot 只接受两个参数
                result = self.robot.set_gripper_value(value, speed)
            self._info(f"夹爪 {label} value={value} speed={speed}% -> {result!r}")
        except Exception as error:                # noqa: BLE001 - 不中止
            self._warn(f"set_gripper_value 异常（继续）: {error}")
        # 真夹爪没有位置反馈，只能按时间等它动作完
        time.sleep(float(robot_conf.get("gripper_settle_s", 1.0)))

    def open_gripper(self):
        self.gripper(int(self.config["robot"].get("gripper_open_value", 100)), "张开")

    def close_gripper(self):
        """往死里夹：直接给最紧的位置 + 最大速度。"""
        self.gripper(int(self.config["robot"].get("gripper_close_value", 0)), "夹紧")

    # ------------------------------------------------------------ 收尾
    def release(self):
        """结束时让电机松力，不做任何"停住"的锁定。"""
        for name in ("release_all_servos", "release_servos"):
            method = getattr(self.robot, name, None)
            if callable(method):
                try:
                    method()
                    self._info(f"已调用 {name}()")
                except Exception as error:        # noqa: BLE001
                    self._warn(f"{name} 异常（忽略）: {error}")
                return
