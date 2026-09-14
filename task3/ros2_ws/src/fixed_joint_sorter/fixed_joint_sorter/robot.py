import time


class RobotError(RuntimeError):
    pass


class JointRobot:
    def __init__(self, cfg, logger=print, client=None):
        self.cfg = cfg
        self.log = logger
        if client is None:
            from pymycobot import MechArmSocket
            client = MechArmSocket(cfg["robot"]["ip"], int(cfg["robot"]["port"]))
        self.client = client

    def _send_async(self, target):
        speed = int(self.cfg["robot"]["arm_speed"])
        try:
            return self.client.send_angles(list(map(float, target)), speed, _async=True)
        except TypeError:
            return self.client.send_angles(list(map(float, target)), speed)

    def move(self, target, label):
        ack = self._send_async(target)
        interval = float(self.cfg["robot"]["command_interval_s"])
        self.log("{}: 命令已发，ACK={}；不读取关节反馈，{:.1f}秒后执行下一步".format(label, ack, interval))
        time.sleep(interval)

    def gripper(self, close):
        # set_gripper_state 不依赖容易丢失的 set_gripper_value 回包。
        flag = 1 if close else 0
        ack = self.client.set_gripper_state(flag, int(self.cfg["robot"]["gripper_speed"]))
        self.log("夹爪{}，ACK={}（不把 ACK=-1 当失败）".format("夹紧" if close else "张开", ack))
        time.sleep(float(self.cfg["robot"]["gripper_settle_s"]))

    def stop(self):
        try:
            self.client.stop()
        except Exception:
            pass
