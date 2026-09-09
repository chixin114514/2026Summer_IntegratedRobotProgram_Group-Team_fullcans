import math

import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from task2_sim.runtime_config import Task2Config


class SafetyMonitor(Node):
    """Command validation and latched SAFE_STOP for Task2."""

    def __init__(self):
        super().__init__('task2_safety_monitor')

        config_dir = get_package_share_directory('task2_sim') + '/config'
        self.config = Task2Config(config_dir)
        common = self.config.communication['common']
        safety = self.config.safety

        self.software_safety_enabled = bool(
            safety.get('motion', {}).get('software_safety_enabled', True)
        )
        self.real_queued_execution = bool(
            self.config.device.get('real', {}).get('queued_execution', True)
        )

        limit_config = safety['joint_limits_deg']
        self.lower_limits = []
        self.upper_limits = []
        for index in range(1, 7):
            joint = limit_config[f'joint{index}']
            self.lower_limits.append(math.radians(float(joint['min'])))
            self.upper_limits.append(math.radians(float(joint['max'])))

        if self.config.is_simulation:
            maximum_step_deg = float(
                safety['motion']['maximum_joint_step_deg_simulation']
            )
        else:
            maximum_step_deg = float(
                safety['motion']['maximum_joint_step_deg_real']
            )
        self.maximum_joint_step = math.radians(maximum_step_deg)

        self.stop_latched = False
        self.stop_reason = ''
        self.previous_command = None

        self.validated_command_pub = self.create_publisher(
            Float64MultiArray,
            common['arm_command_topic'],
            10,
        )
        self.safety_state_pub = self.create_publisher(
            String,
            common['safety_state_topic'],
            10,
        )
        self.safety_stop_pub = self.create_publisher(
            Bool,
            common['safety_stop_topic'],
            10,
        )

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            common['requested_arm_command_topic'],
            self.command_callback,
            10,
        )
        self.reset_sub = self.create_subscription(
            Bool,
            common['safety_reset_topic'],
            self.reset_callback,
            10,
        )
        self.task_fault_sub = self.create_subscription(
            String,
            common['task_fault_topic'],
            self.task_fault_callback,
            10,
        )
        self.task_state_sub = self.create_subscription(
            String,
            common['task_state_topic'],
            self.task_state_callback,
            10,
        )

        self.publish_state('OK')
        self.get_logger().info('Safety monitor started.')
        self.get_logger().info(f'Mode: {self.config.mode_name()}')
        if self.config.is_real_robot and self.real_queued_execution:
            self.get_logger().info(
                'REAL QUEUED SAFETY: hard joint limits + measured feedback + '
                'goal timeout; per-target 3 degree step check is not used.'
            )
        else:
            self.get_logger().info(
                f'Maximum command step: {maximum_step_deg:.1f} deg'
            )

    def publish_state(self, state):
        state_msg = String()
        state_msg.data = str(state)
        self.safety_state_pub.publish(state_msg)

        stop_msg = Bool()
        stop_msg.data = self.stop_latched
        self.safety_stop_pub.publish(stop_msg)

    def trigger_stop(self, reason):
        if not self.software_safety_enabled:
            self.get_logger().warn(
                'SOFTWARE_SAFE_STOP_IGNORED: ' + str(reason)
            )
            self.stop_latched = False
            self.stop_reason = ''
            self.publish_state('OK')
            return

        self.stop_latched = True
        self.stop_reason = str(reason)
        message = 'SAFE_STOP: ' + self.stop_reason
        self.get_logger().error(message)
        self.publish_state(message)

    def reset_callback(self, message):
        if not message.data:
            return
        self.stop_latched = False
        self.stop_reason = ''
        self.previous_command = None
        self.get_logger().warn('Safety monitor latch reset.')
        self.publish_state('OK')

    def task_fault_callback(self, message):
        reason = message.data.strip() or 'UNSPECIFIED_TASK_FAULT'
        self.trigger_stop(reason)

    def task_state_callback(self, message):
        state = message.data.strip()

        # Keep the historical simulation step-reference reset so Gazebo's
        # proven interpolated trajectory behaviour is unchanged. In formal
        # real queued mode a complete goal is intentionally allowed to be
        # farther than 3 degrees from the previous goal because firmware
        # performs the physical interpolation; safety is provided by hard
        # limits and the measured-feedback goal monitor.
        if self.config.is_simulation and state in (
            'READY',
            'STARTED',
            'COMPLETED',
            'HOME',
            'A_SAFE',
            'A_PREGRASP',
            'A_PICK',
            'A_LIFT',
            'B_SAFE',
            'B_PLACE',
            'B_LIFT',
            'RETURN_HOME',
        ):
            self.previous_command = None
        elif self.config.is_real_robot and state in (
            'READY',
            'STARTED',
            'COMPLETED',
            'RESET_HOME',
        ):
            self.previous_command = None

    def command_callback(self, message):
        if self.stop_latched:
            self.get_logger().warn(
                'Arm command rejected: safety stop is latched.'
            )
            return

        values = [float(value) for value in message.data]
        if len(values) != 6:
            self.trigger_stop(
                f'INVALID_COMMAND_LENGTH expected=6 actual={len(values)}'
            )
            return
        if not all(math.isfinite(value) for value in values):
            self.trigger_stop('NON_FINITE_JOINT_COMMAND')
            return

        if not self.software_safety_enabled:
            self.forward(values)
            return

        for index, (value, lower, upper) in enumerate(
            zip(values, self.lower_limits, self.upper_limits),
            start=1,
        ):
            if value < lower or value > upper:
                self.trigger_stop(
                    'JOINT_LIMIT '
                    f'joint={index} value_deg={math.degrees(value):.2f} '
                    f'allowed_deg=[{math.degrees(lower):.2f},'
                    f'{math.degrees(upper):.2f}]'
                )
                return

        enforce_step = not (
            self.config.is_real_robot and self.real_queued_execution
        )
        if enforce_step and self.previous_command is not None:
            for index, (current, previous) in enumerate(
                zip(values, self.previous_command),
                start=1,
            ):
                delta = abs(current - previous)
                if delta > self.maximum_joint_step:
                    self.trigger_stop(
                        'JOINT_STEP_TOO_LARGE '
                        f'joint={index} '
                        f'delta_deg={math.degrees(delta):.2f} '
                        f'max_deg={math.degrees(self.maximum_joint_step):.2f}'
                    )
                    return

        self.forward(values)

    def forward(self, values):
        validated = Float64MultiArray()
        validated.data = list(values)
        self.validated_command_pub.publish(validated)
        self.previous_command = list(values)
        self.publish_state('OK')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
