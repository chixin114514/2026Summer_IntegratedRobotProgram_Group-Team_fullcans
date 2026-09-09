import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from task2_sim.runtime_config import Task2Config


class RealRobotDriver(Node):
    """MechArmSocket adapter used by the formal real-robot launch chain."""

    def __init__(self):
        super().__init__('task2_real_robot_driver')

        config_dir = get_package_share_directory('task2_sim') + '/config'
        self.config = Task2Config(config_dir)
        if not self.config.is_real_robot:
            raise RuntimeError(
                'RealRobotDriver may only run when device mode = 1.'
            )

        common = self.config.communication['common']
        communication = self.config.communication['real']
        device = self.config.device['real']

        self.driver_type = str(
            device.get('driver_type', 'pymycobot_serial')
        )
        self.device = str(device.get('serial_device', '/dev/ttyUSB0'))
        self.baudrate = int(device.get('baudrate', 115200))
        self.robot_ip = str(device.get('robot_ip', '192.168.50.2'))
        self.robot_port = int(device.get('robot_port', 9000))
        self.arm_speed = int(device['arm_speed_percent'])
        self.gripper_speed = int(device['gripper_speed_percent'])
        self.feedback_period = float(device['feedback_period_s'])
        self.queued_execution = bool(device.get('queued_execution', True))
        self.feedback_max_age_s = max(
            0.05,
            float(device.get('real_feedback_max_age_s', 0.75)),
        )
        self.feedback_fault_samples = max(
            1,
            int(math.ceil(self.feedback_max_age_s / self.feedback_period)),
        )

        if not self.queued_execution:
            raise RuntimeError(
                'Formal real mode requires queued_execution=true. '
                'A streaming implementation must use fresh_mode=1 and a '
                'separate control design.'
            )

        self.stop_latched = False
        self.hardware_ready = False
        self.consecutive_feedback_errors = 0
        self.invalid_feedback_samples = 0
        self.last_gripper_device_value = None

        # Hardware readiness is deliberately separate from ROS
        # connection readiness. get_angles() may work while the
        # actuator core is paused, unpowered or has a disabled
        # servo, in which case send_angles() may not move.
        self.hardware_ready = False

        try:
            if self.driver_type == 'pymycobot_socket':
                try:
                    from pymycobot import MechArmSocket
                except ImportError:
                    from pymycobot.mecharmsocket import MechArmSocket
            else:
                try:
                    from pymycobot.mecharm270 import MechArm270
                except ImportError:
                    from pymycobot import MechArm270
        except ImportError as error:
            raise RuntimeError(
                'pymycobot is required for mode=1.'
            ) from error

        try:
            if self.driver_type == 'pymycobot_socket':
                self.get_logger().info(
                    'Connecting to mechArm 270 Pi through TCP socket: '
                    f'{self.robot_ip}:{self.robot_port}'
                )
                self.robot = MechArmSocket(self.robot_ip, self.robot_port)
            else:
                self.get_logger().info(
                    f'Connecting to MechArm 270 serial: '
                    f'{self.device} @ {self.baudrate}'
                )
                self.robot = MechArm270(self.device, self.baudrate)
        except Exception as error:
            raise RuntimeError(
                'Unable to open MechArm 270 communication: '
                + str(error)
            ) from error

        # Formal queued mode sends exactly one complete target and waits for
        # measured arrival before TaskManager may publish the next target.
        # Therefore fresh_mode=0 cannot accumulate a multi-state backlog.
        try:
            if hasattr(self.robot, 'set_fresh_mode'):
                self.robot.set_fresh_mode(0)
                self.get_logger().info(
                    'Real robot fresh mode = 0 (ordered one-goal execution)'
                )
        except Exception as error:
            self.get_logger().warn('Unable to set fresh mode: ' + str(error))

        self.joint_state_pub = self.create_publisher(
            JointState,
            communication['joint_state_topic'],
            10,
        )
        self.connection_pub = self.create_publisher(
            String,
            communication['connection_state_topic'],
            10,
        )
        self.task_fault_pub = self.create_publisher(
            String,
            common['task_fault_topic'],
            10,
        )

        self.arm_sub = self.create_subscription(
            Float64MultiArray,
            communication['joint_command_topic'],
            self.arm_command_callback,
            1,
        )
        self.gripper_sub = self.create_subscription(
            Float64,
            communication['gripper_command_topic'],
            self.gripper_command_callback,
            1,
        )
        self.safety_sub = self.create_subscription(
            Bool,
            common['safety_stop_topic'],
            self.safety_callback,
            10,
        )
        self.safety_reset_sub = self.create_subscription(
            Bool,
            common['safety_reset_topic'],
            self.safety_reset_callback,
            10,
        )
        self.external_estop_sub = self.create_subscription(
            Bool,
            communication['emergency_stop_topic'],
            self.external_estop_callback,
            10,
        )

        self.timer = self.create_timer(
            self.feedback_period,
            self.read_joint_state,
        )
        self.publish_connection('CONNECTED')
        self.get_logger().warn('REAL ROBOT MODE: LOW-SPEED OPERATION')
        self.get_logger().warn(f'Arm speed = {self.arm_speed}%')
        self.get_logger().info(
            f'Feedback fault threshold = {self.feedback_fault_samples} '
            f'samples ({self.feedback_max_age_s:.2f}s max age)'
        )

    def optional_robot_call(
        self,
        name,
        *args,
    ):
        method = getattr(
            self.robot,
            name,
            None,
        )

        if not callable(method):
            return None

        return method(*args)

    def prepare_hardware_for_motion(self):
        """Bring MechArm into a commandable position-control state."""

        # If we previously completed full preparation, perform
        # one cheap servo check before every new target. This
        # catches a joint (especially J3) dropping out between
        # motion states.
        if self.hardware_ready:

            try:
                all_servos = (
                    self.optional_robot_call(
                        'is_all_servo_enable'
                    )
                )

                if all_servos in (
                    None,
                    1,
                    True,
                ):
                    return True

                self.get_logger().warn(
                    'REAL_HW_RECHECK: servo enable state '
                    f'changed to {all_servos!r}; '
                    'running full preparation.'
                )

                self.hardware_ready = False

            except Exception as error:
                self.get_logger().warn(
                    'REAL_HW_RECHECK_EXCEPTION: '
                    + str(error)
                )

                self.hardware_ready = False

        self.get_logger().warn(
            'REAL_HW_PREPARE: validating physical '
            'MechArm controller before motion.'
        )

        # -----------------------------------------------------
        # Controller / Atom connection
        # -----------------------------------------------------

        connected = self.optional_robot_call(
            'is_controller_connected'
        )

        if connected not in (
            None,
            1,
            True,
        ):
            raise RuntimeError(
                'controller_not_connected '
                f'status={connected!r}'
            )

        # -----------------------------------------------------
        # Power
        # -----------------------------------------------------

        power = self.optional_robot_call(
            'is_power_on'
        )

        if power == 0:

            self.get_logger().warn(
                'REAL_HW_PREPARE: power is OFF; '
                'calling power_on().'
            )

            result = self.optional_robot_call(
                'power_on'
            )

            self.get_logger().warn(
                'REAL_HW_POWER_ON '
                f'result={result!r}'
            )

            time.sleep(0.30)

            power = self.optional_robot_call(
                'is_power_on'
            )

        if power not in (
            None,
            1,
            True,
        ):
            raise RuntimeError(
                'robot_power_not_ready '
                f'status={power!r}'
            )

        # -----------------------------------------------------
        # Free / drag mode must be OFF for position commands.
        # -----------------------------------------------------

        free_mode = self.optional_robot_call(
            'is_free_mode'
        )

        if free_mode == 1:

            self.get_logger().warn(
                'REAL_HW_PREPARE: free mode is ON; '
                'disabling free mode.'
            )

            self.optional_robot_call(
                'set_free_mode',
                0,
            )

            time.sleep(0.15)

        # -----------------------------------------------------
        # If a previous run left the controller paused,
        # release that state before the new command.
        # -----------------------------------------------------

        paused = self.optional_robot_call(
            'is_paused'
        )

        if paused == 1:

            self.get_logger().warn(
                'REAL_HW_PREPARE: controller is PAUSED; '
                'calling resume().'
            )

            result = self.optional_robot_call(
                'resume'
            )

            self.get_logger().warn(
                'REAL_HW_RESUME '
                f'result={result!r}'
            )

            time.sleep(0.20)

        # -----------------------------------------------------
        # Servo enable state.
        #
        # This directly addresses the earlier J3 symptom.
        # A readable angle does NOT prove the servo torque /
        # actuator is enabled.
        # -----------------------------------------------------

        servo_method = getattr(
            self.robot,
            'is_servo_enable',
            None,
        )

        servo_states = []

        if callable(servo_method):

            for servo_id in range(
                1,
                7,
            ):

                status = servo_method(
                    servo_id
                )

                if status != 1:

                    self.get_logger().warn(
                        'REAL_HW_SERVO_ENABLE '
                        f'joint={servo_id} '
                        f'before={status!r}'
                    )

                    focus = getattr(
                        self.robot,
                        'focus_servo',
                        None,
                    )

                    if callable(focus):
                        focus(
                            servo_id
                        )

                    else:
                        focus_all = getattr(
                            self.robot,
                            'focus_all_servos',
                            None,
                        )

                        if callable(focus_all):
                            focus_all()

                    time.sleep(0.15)

                    status = servo_method(
                        servo_id
                    )

                servo_states.append(
                    status
                )

                if status != 1:
                    raise RuntimeError(
                        'servo_not_enabled '
                        f'joint={servo_id} '
                        f'status={status!r}'
                    )

        else:

            all_servos = (
                self.optional_robot_call(
                    'is_all_servo_enable'
                )
            )

            if all_servos not in (
                None,
                1,
                True,
            ):

                self.get_logger().warn(
                    'REAL_HW_PREPARE: not all servos '
                    'are enabled; focusing all servos.'
                )

                self.optional_robot_call(
                    'focus_all_servos'
                )

                time.sleep(0.30)

                all_servos = (
                    self.optional_robot_call(
                        'is_all_servo_enable'
                    )
                )

                if all_servos not in (
                    None,
                    1,
                    True,
                ):
                    raise RuntimeError(
                        'all_servos_not_enabled '
                        f'status={all_servos!r}'
                    )

        # -----------------------------------------------------
        # Do NOT automatically clear genuine robot errors.
        # A collision / limit error must remain a real fault.
        # -----------------------------------------------------

        error_info = self.optional_robot_call(
            'get_error_information'
        )

        if error_info not in (
            None,
            0,
            [],
            [0, 0, 0, 0, 0, 0],
        ):
            raise RuntimeError(
                'robot_error_information '
                f'value={error_info!r}'
            )

        # Formal one-goal-at-a-time operation.
        self.optional_robot_call(
            'set_fresh_mode',
            0,
        )

        self.hardware_ready = True

        self.get_logger().warn(
            'REAL_HW_READY '
            f'controller={connected!r} '
            f'power={power!r} '
            f'free_mode={free_mode!r} '
            f'paused={paused!r} '
            f'servos={servo_states!r} '
            f'error={error_info!r}'
        )

        return True

    def publish_connection(self, state):
        message = String()
        message.data = str(state)
        self.connection_pub.publish(message)

    def publish_fault(self, reason):
        self.get_logger().error(str(reason))
        message = String()
        message.data = str(reason)
        self.task_fault_pub.publish(message)

    def communication_fault(self, reason):
        self.publish_connection('COMMUNICATION_ERROR')
        self.perform_stop()
        self.publish_fault(reason)

    def arm_command_callback(self, message):
        if self.stop_latched:
            self.get_logger().warn(
                'REAL_ARM_COMMAND_REJECTED: driver stop latch is active.'
            )
            return
        if len(message.data) != 6:
            self.communication_fault('REAL_ARM_INVALID_COMMAND_LENGTH')
            return

        try:
            values = [
                float(value)
                for value in message.data
            ]

            if not all(
                math.isfinite(value)
                for value in values
            ):
                raise ValueError(
                    'non-finite joint command'
                )

            # The ROS path being alive is not sufficient.
            # Prepare the actual actuator state first.
            self.prepare_hardware_for_motion()

            degrees = [
                math.degrees(value)
                for value in values
            ]

            self.get_logger().warn(
                'REAL_ARM_TX '
                'target_deg=['
                + ', '.join(
                    f'{value:.2f}'
                    for value in degrees
                )
                + '] '
                + f'speed={self.arm_speed}%'
            )

            result = self.robot.send_angles(
                degrees,
                self.arm_speed,
            )

            self.get_logger().info(
                'REAL_ARM_TX_RETURN '
                f'value={result!r}'
            )

            if result in (
                0,
                -1,
                False,
            ):
                raise RuntimeError(
                    'send_angles rejected '
                    f'result={result!r}'
                )

        except Exception as error:
            self.hardware_ready = False

            self.communication_fault(
                'REAL_ARM_COMMUNICATION_FAILURE: '
                + str(error)
            )

    def gripper_command_callback(self, message):
        if self.stop_latched:
            self.get_logger().warn(
                'REAL_GRIPPER_COMMAND_REJECTED: driver stop latch is active.'
            )
            return

        task_gripper = self.config.task['gripper']
        open_position = float(task_gripper['open_position'])
        closed_position = float(task_gripper['closed_position'])
        command = float(message.data)
        span = max(1e-6, closed_position - open_position)
        closed_fraction = max(
            0.0,
            min(1.0, (command - open_position) / span),
        )
        device_value = int(round(100.0 * (1.0 - closed_fraction)))

        if (
            self.last_gripper_device_value is not None
            and device_value == self.last_gripper_device_value
        ):
            return

        try:
            self.robot.set_gripper_value(
                device_value,
                self.gripper_speed,
                1,
            )
            self.last_gripper_device_value = device_value
        except Exception as error:
            self.communication_fault(
                'REAL_GRIPPER_COMMUNICATION_FAILURE: ' + str(error)
            )

    def feedback_failure(self, reason, invalid=False):
        if invalid:
            self.invalid_feedback_samples += 1
            count = self.invalid_feedback_samples
        else:
            self.consecutive_feedback_errors += 1
            count = self.consecutive_feedback_errors

        self.publish_connection('FEEDBACK_WAIT')
        if count == 1 or count == self.feedback_fault_samples:
            self.get_logger().warn(reason)

        if count >= self.feedback_fault_samples:
            self.communication_fault('REAL_ROBOT_FEEDBACK_FAILURE: ' + reason)

    def read_joint_state(self):
        if self.stop_latched:
            return

        try:
            raw_angles = self.robot.get_angles()
        except Exception as error:
            self.feedback_failure('REAL_FEEDBACK_EXCEPTION: ' + str(error))
            return

        if not isinstance(raw_angles, (list, tuple)):
            self.feedback_failure(
                f'REAL_FEEDBACK_INVALID raw={raw_angles!r}',
                invalid=True,
            )
            return
        if len(raw_angles) != 6:
            self.feedback_failure(
                f'REAL_FEEDBACK_INVALID expected=6 actual={len(raw_angles)}',
                invalid=True,
            )
            return

        try:
            angles = [float(value) for value in raw_angles]
        except (TypeError, ValueError) as error:
            self.feedback_failure(
                'REAL_FEEDBACK_INVALID non-numeric: ' + str(error),
                invalid=True,
            )
            return

        if not all(math.isfinite(value) for value in angles):
            self.feedback_failure(
                'REAL_FEEDBACK_INVALID non-finite angle',
                invalid=True,
            )
            return

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [
            'joint1_to_base',
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
        ]
        message.position = [math.radians(value) for value in angles]
        self.joint_state_pub.publish(message)

        if self.invalid_feedback_samples or self.consecutive_feedback_errors:
            self.get_logger().info('REAL_FEEDBACK_RECOVERED')
        self.invalid_feedback_samples = 0
        self.consecutive_feedback_errors = 0
        self.publish_connection('CONNECTED')

    def perform_stop(self):
        if self.stop_latched:
            return
        self.stop_latched = True
        self.hardware_ready = False

        try:
            if hasattr(self.robot, 'stop'):
                self.robot.stop()
            elif hasattr(self.robot, 'pause'):
                self.robot.pause()
        except Exception as error:
            self.get_logger().error('REAL_STOP_EXCEPTION: ' + str(error))

        self.publish_connection('STOPPED')
        self.get_logger().error('REAL ROBOT STOPPED')

    def safety_callback(self, message):
        if message.data:
            self.perform_stop()

    def safety_reset_callback(self, message):
        if not message.data:
            return
        if not self.stop_latched:
            self.get_logger().info('Driver safety reset: latch already clear.')
            return

        try:
            if hasattr(self.robot, 'resume'):
                self.robot.resume()
        except Exception as error:
            self.publish_fault('REAL_ROBOT_RESUME_FAILURE: ' + str(error))
            return

        self.stop_latched = False
        self.consecutive_feedback_errors = 0
        self.invalid_feedback_samples = 0
        self.last_gripper_device_value = None
        self.publish_connection('CONNECTED')
        self.get_logger().warn('Real robot driver safety latch reset.')

    def external_estop_callback(self, message):
        if message.data:
            self.perform_stop()
            self.publish_fault('EXTERNAL_EMERGENCY_STOP')


def main(args=None):
    rclpy.init(args=args)
    node = RealRobotDriver()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.perform_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
