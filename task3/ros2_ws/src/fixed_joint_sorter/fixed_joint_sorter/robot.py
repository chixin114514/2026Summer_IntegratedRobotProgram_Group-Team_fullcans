import time


class RobotError(RuntimeError):
    pass


def run_pick_place_cycle(robot, station, drop_point, home, prefix):
    """Run one fixed-angle pick/place cycle in the required order."""
    robot.gripper(close=True, label=prefix + "_close_before_approach")
    robot.move(station["approach_angles"], prefix + "_approach")
    robot.gripper(close=False, label=prefix + "_open_before_pick")
    robot.move(station["pick_angles"], prefix + "_pick")
    robot.gripper(close=True, label=prefix + "_close_at_pick")
    robot.move(station["retreat_angles"], prefix + "_retreat")
    robot.move(drop_point["place_angles"], prefix + "_drop_place")
    robot.gripper(close=False, label=prefix + "_open_at_drop")
    robot.move(home, prefix + "_return_home")


class JointRobot:
    def __init__(self, cfg, logger=print, client=None):
        self.cfg = cfg
        self.log = logger
        if client is None:
            from pymycobot import MechArmSocket
            client = MechArmSocket(cfg["robot"]["ip"], int(cfg["robot"]["port"]))
        self.client = client
        self.log("queue_mode: client.set_fresh_mode(0)")
        try:
            self.client.set_fresh_mode(0)
        except Exception as exc:
            raise RobotError("无法设置插补队列模式: {}".format(exc)) from exc

    def _send_async(self, target, label):
        speed = int(self.cfg["robot"]["arm_speed"])
        coords = list(map(float, target))
        try:
            self.log("{}: client.send_coords(coords={}, speed={}, _async=True)".format(label, coords, speed))
            return self.client.send_coords(coords, speed, _async=True)
        except TypeError:
            self.log("{}: client.send_coords(coords={}, speed={})".format(label, coords, speed))
            return self.client.send_coords(coords, speed)

    def _wait_for_next_command(self, fallback_key):
        robot_cfg = self.cfg["robot"]
        delay = float(robot_cfg.get("next_command_delay_s", robot_cfg.get(fallback_key, 0.0)))
        if delay < 0:
            raise RobotError("next_command_delay_s 不能小于 0")
        time.sleep(delay)

    def move(self, target, label):
        self._send_async(target, label)
        self._wait_for_next_command("command_interval_s")

    def gripper(self, close, label="gripper"):
        # set_gripper_state 不依赖容易丢失的 set_gripper_value 回包。
        flag = 1 if close else 0
        speed = int(self.cfg["robot"]["gripper_speed"])
        self.log("{}: client.set_gripper_state(flag={}, speed={})".format(label, flag, speed))
        self.client.set_gripper_state(flag, speed)
        self._wait_for_next_command("gripper_settle_s")

    def stop(self):
        self.log("stop: client.stop()")
        try:
            self.client.stop()
        except Exception:
            pass
