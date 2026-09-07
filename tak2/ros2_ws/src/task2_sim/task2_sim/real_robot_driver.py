import math

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import (
    Bool,
    Float64,
    Float64MultiArray,
    String,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class RealRobotDriver(Node):

    def __init__(self):

        super().__init__(
            'task2_real_robot_driver'
        )

        config_dir = (
            get_package_share_directory(
                'task2_sim'
            )
            + '/config'
        )

        self.config = Task2Config(
            config_dir
        )

        if not self.config.is_real_robot:

            raise RuntimeError(
                'RealRobotDriver may only run '
                'when device mode = 1.'
            )

        common = (
            self.config.communication[
                'common'
            ]
        )

        communication = (
            self.config.communication[
                'real'
            ]
        )

        device = (
            self.config.device[
                'real'
            ]
        )

        # -----------------------------------------------------
        # Real communication backend
        #
        # mechArm 270 Pi:
        #
        # Jetson
        #   -> Wi-Fi / TCP
        #   -> Server_270.py
        #   -> /dev/ttyAMA0
        #   -> robot controller
        # -----------------------------------------------------

        self.driver_type = str(
            device.get(
                'driver_type',
                'pymycobot_serial',
            )
        )

        self.device = str(
            device.get(
                'serial_device',
                '/dev/ttyUSB0',
            )
        )

        self.baudrate = int(
            device.get(
                'baudrate',
                115200,
            )
        )

        self.robot_ip = str(
            device.get(
                'robot_ip',
                '10.238.238.133',
            )
        )

        self.robot_port = int(
            device.get(
                'robot_port',
                9000,
            )
        )

        self.arm_speed = int(
            device[
                'arm_speed_percent'
            ]
        )

        self.gripper_speed = int(
            device[
                'gripper_speed_percent'
            ]
        )

        self.command_period = float(
            device.get(
                'command_period_s',
                0.10,
            )
        )


        self.final_command_resend = float(
            device.get(
                'final_command_resend_s',
                0.80,
            )
        )

        self.feedback_period = float(
            device[
                'feedback_period_s'
            ]
        )

        self.connection_timeout = float(
            device[
                'timeout_s'
            ]
        )

        self.stop_latched = False

        self.last_feedback_success = None

        self.consecutive_feedback_errors = 0

        # MechArmSocket / pymycobot may occasionally return
        # an integer sentinel such as -1 instead of a six-angle
        # list when a feedback response is not ready.
        #
        # This is NOT immediately considered a communication
        # failure. The sample is simply discarded.
        self.invalid_feedback_samples = 0

        # -----------------------------------------------------
        # Latest-command buffer.
        #
        # Do NOT queue historical trajectory commands for the
        # physical arm. Only the newest desired joint target
        # matters.
        # -----------------------------------------------------

        self.latest_arm_degrees = None

        self.last_sent_arm_degrees = None

        self.last_arm_send_time = None

        self.arm_command_pending = False

        # -----------------------------------------------------
        # Load pymycobot only in REAL mode.
        # -----------------------------------------------------

        try:

            if (
                self.driver_type
                ==
                'pymycobot_socket'
            ):

                try:

                    from pymycobot import (
                        MechArmSocket,
                    )

                except ImportError:

                    from pymycobot.mecharmsocket import (
                        MechArmSocket,
                    )

            else:

                try:

                    from pymycobot.mecharm270 import (
                        MechArm270,
                    )

                except ImportError:

                    from pymycobot import (
                        MechArm270,
                    )

        except ImportError as error:

            raise RuntimeError(
                'pymycobot is required for mode=1. '
                'Install it before real-robot operation.'
            ) from error


        try:

            if (
                self.driver_type
                ==
                'pymycobot_socket'
            ):

                self.get_logger().info(
                    'Connecting to mechArm 270 Pi '
                    'through TCP socket: '
                    f'{self.robot_ip}:{self.robot_port}'
                )

                self.robot = MechArmSocket(
                    self.robot_ip,
                    self.robot_port,
                )

            else:

                self.get_logger().info(
                    'Connecting to MechArm 270 serial: '
                    f'{self.device} @ {self.baudrate}'
                )

                self.robot = MechArm270(
                    self.device,
                    self.baudrate,
                )

        except Exception as error:

            raise RuntimeError(
                'Unable to open MechArm 270 '
                f'communication: {error}'
            )

        # -----------------------------------------------------
        # Real-time command mode.
        #
        # TaskManager streams smooth intermediate targets.
        # Execute the newest command instead of building a
        # command queue on the physical robot.
        # -----------------------------------------------------

        try:

            if hasattr(
                self.robot,
                'set_fresh_mode',
            ):

                self.robot.set_fresh_mode(
                    1
                )

                self.get_logger().info(
                    'Real robot fresh mode = 1 '
                    '(latest command first)'
                )

        except Exception as error:

            self.get_logger().warn(
                'Unable to set fresh mode: '
                + str(error)
            )

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------

        self.joint_state_pub = (
            self.create_publisher(
                JointState,
                communication[
                    'joint_state_topic'
                ],
                10,
            )
        )

        self.connection_pub = (
            self.create_publisher(
                String,
                communication[
                    'connection_state_topic'
                ],
                10,
            )
        )

        self.task_fault_pub = (
            self.create_publisher(
                String,
                common[
                    'task_fault_topic'
                ],
                10,
            )
        )

        # -----------------------------------------------------
        # Commands produced by arm/gripper interfaces
        # -----------------------------------------------------

        self.arm_sub = (
            self.create_subscription(
                Float64MultiArray,
                communication[
                    'joint_command_topic'
                ],
                self.arm_command_callback,

                # Real robot:
                # KEEP LAST 1.
                #
                # Never allow stale joint targets to build up
                # in the ROS subscriber queue.
                1,
            )
        )

        self.gripper_sub = (
            self.create_subscription(
                Float64,
                communication[
                    'gripper_command_topic'
                ],
                self.gripper_command_callback,
                10,
            )
        )

        # -----------------------------------------------------
        # Safety stop
        # -----------------------------------------------------

        self.safety_sub = (
            self.create_subscription(
                Bool,
                common[
                    'safety_stop_topic'
                ],
                self.safety_callback,
                10,
            )
        )

        self.external_estop_sub = (
            self.create_subscription(
                Bool,
                communication[
                    'emergency_stop_topic'
                ],
                self.external_estop_callback,
                10,
            )
        )

        # -----------------------------------------------------
        # Real communication scheduling.
        #
        # Commands and feedback share the same TCP/Pi path.
        #
        # Command timer:
        #   sends only the newest command.
        #
        # Feedback timer:
        #   periodically reads measured joint angles.
        # -----------------------------------------------------

        self.arm_command_timer = self.create_timer(
            self.command_period,
            self.send_latest_arm_command,
        )

        self.feedback_timer = self.create_timer(
            self.feedback_period,
            self.read_joint_state,
        )

        self.publish_connection(
            'CONNECTED'
        )

        self.get_logger().warn(
            'REAL ROBOT MODE: LOW-SPEED OPERATION'
        )

        self.get_logger().warn(
            f'Arm speed = {self.arm_speed}%'
        )


        self.get_logger().info(
            f'Command period = '
            f'{self.command_period:.2f} s'
        )

        self.get_logger().info(
            f'Feedback period = '
            f'{self.feedback_period:.2f} s'
        )

    # ========================================================
    # Helpers
    # ========================================================

    def publish_connection(
        self,
        state,
    ):

        message = String()

        message.data = str(
            state
        )

        self.connection_pub.publish(
            message
        )

    def publish_fault(
        self,
        reason,
    ):

        self.get_logger().error(
            str(reason)
        )

        message = String()

        message.data = str(
            reason
        )

        self.task_fault_pub.publish(
            message
        )

    # ========================================================
    # Arm
    # ========================================================

    def arm_command_callback(
        self,
        message,
    ):

        if self.stop_latched:

            return

        if len(
            message.data
        ) != 6:

            self.publish_fault(
                'REAL_ARM_INVALID_COMMAND_LENGTH'
            )

            return

        degrees = [
            math.degrees(
                float(value)
            )
            for value in message.data
        ]

        # -----------------------------------------------------
        # IMPORTANT:
        #
        # Do NOT call send_angles() here.
        #
        # The ROS callback must remain fast so new messages
        # cannot accumulate behind a blocking TCP operation.
        #
        # Simply replace the previous target with the newest.
        # -----------------------------------------------------

        self.latest_arm_degrees = list(
            degrees
        )

        self.arm_command_pending = True


    def send_latest_arm_command(
        self,
    ):

        if self.stop_latched:

            return

        if (
            not self.arm_command_pending
            or
            self.latest_arm_degrees is None
        ):

            return

        degrees = list(
            self.latest_arm_degrees
        )

        # Mark consumed BEFORE the blocking network call.
        #
        # With queue depth = 1, when the executor becomes free
        # again it receives only the newest waiting command.
        self.arm_command_pending = False


        # -----------------------------------------------------
        # Do not repeatedly transmit an identical final target.
        #
        # TaskManager intentionally keeps publishing the final
        # target while waiting for measured convergence.
        # -----------------------------------------------------

        now_s = (
            self.get_clock()
            .now()
            .nanoseconds
            *
            1e-9
        )

        if (
            self.last_sent_arm_degrees
            is not None
        ):

            maximum_change = max(
                abs(
                    current
                    -
                    previous
                )
                for current, previous
                in zip(
                    degrees,
                    self.last_sent_arm_degrees,
                )
            )

            # -------------------------------------------------
            # Same target:
            #
            # Do not send at 10 Hz forever, but also do NOT
            # suppress it permanently.
            #
            # Re-send every ~0.8 s while TaskManager is still
            # requesting the final target.
            # -------------------------------------------------

            if maximum_change < 0.01:

                if (
                    self.last_arm_send_time
                    is not None
                    and
                    (
                        now_s
                        -
                        self.last_arm_send_time
                    )
                    <
                    self.final_command_resend
                ):

                    return


        try:

            self.robot.send_angles(
                degrees,
                self.arm_speed,
            )

            self.last_sent_arm_degrees = list(
                degrees
            )

            self.last_arm_send_time = now_s

        except Exception as error:

            self.publish_connection(
                'COMMUNICATION_ERROR'
            )

            self.publish_fault(
                'REAL_ARM_COMMUNICATION_FAILURE: '
                +
                str(error)
            )


    # ========================================================
    # Gripper
    # ========================================================

    def gripper_command_callback(
        self,
        message,
    ):

        if self.stop_latched:

            return

        task_gripper = (
            self.config.task[
                'gripper'
            ]
        )

        open_position = float(
            task_gripper[
                'open_position'
            ]
        )

        closed_position = float(
            task_gripper[
                'closed_position'
            ]
        )

        command = float(
            message.data
        )

        span = max(
            1e-6,
            closed_position
            -
            open_position,
        )

        closed_fraction = (
            command
            -
            open_position
        ) / span

        closed_fraction = max(
            0.0,
            min(
                1.0,
                closed_fraction,
            ),
        )

        # pymycobot gripper convention:
        # 100 = open
        # 0   = closed

        device_value = int(
            round(
                100.0
                *
                (
                    1.0
                    -
                    closed_fraction
                )
            )
        )

        try:

            self.robot.set_gripper_value(
                device_value,
                self.gripper_speed,
                1,  # Adaptive Gripper
            )

        except Exception as error:

            self.publish_fault(
                'REAL_GRIPPER_COMMUNICATION_FAILURE: '
                +
                str(error)
            )

    # ========================================================
    # Feedback
    # ========================================================

    def read_joint_state(self):

        if self.stop_latched:

            return


        # =====================================================
        # Read raw pymycobot feedback
        # =====================================================

        try:

            angles = (
                self.robot.get_angles()
            )

        except Exception as error:

            # -------------------------------------------------
            # A real socket / transport exception.
            # -------------------------------------------------

            self.consecutive_feedback_errors += 1

            self.publish_connection(
                'FEEDBACK_ERROR'
            )

            self.get_logger().warn(
                'REAL_FEEDBACK_EXCEPTION: '
                +
                str(error)
            )

            if (
                self.consecutive_feedback_errors
                >=
                3
            ):

                self.publish_fault(
                    'REAL_ROBOT_FEEDBACK_FAILURE: '
                    +
                    str(error)
                )

            return


        # =====================================================
        # IMPORTANT FIX
        #
        # pymycobot may return:
        #
        #     -1
        #
        # or another non-list sentinel when a reply is not
        # available yet.
        #
        # NEVER call len() before checking the type.
        # =====================================================

        if not isinstance(
            angles,
            (
                list,
                tuple,
            ),
        ):

            self.invalid_feedback_samples += 1

            self.publish_connection(
                'FEEDBACK_WAIT'
            )

            # Avoid flooding the terminal.
            if (
                self.invalid_feedback_samples == 1
                or
                self.invalid_feedback_samples == 5
                or
                self.invalid_feedback_samples % 10 == 0
            ):

                self.get_logger().warn(
                    'REAL_FEEDBACK_WAIT: '
                    f'non-list feedback={angles!r} '
                    f'samples={self.invalid_feedback_samples}'
                )

            # This sample is simply unavailable.
            #
            # Do NOT SAFE_STOP here.
            # TaskManager has its own measured-feedback
            # progress / stall watchdog.
            return


        # =====================================================
        # Sequence must contain exactly six joints
        # =====================================================

        if len(
            angles
        ) != 6:

            self.invalid_feedback_samples += 1

            self.publish_connection(
                'FEEDBACK_WAIT'
            )

            self.get_logger().warn(
                'REAL_FEEDBACK_WAIT: '
                f'invalid feedback length='
                f'{len(angles)} '
                f'raw={angles!r}'
            )

            return


        # =====================================================
        # Validate every returned angle
        # =====================================================

        try:

            angle_values = [
                float(value)
                for value in angles
            ]

        except (
            TypeError,
            ValueError,
        ) as error:

            self.invalid_feedback_samples += 1

            self.publish_connection(
                'FEEDBACK_WAIT'
            )

            self.get_logger().warn(
                'REAL_FEEDBACK_WAIT: '
                f'non-numeric feedback='
                f'{angles!r}: {error}'
            )

            return


        # =====================================================
        # Valid feedback
        # =====================================================

        message = JointState()

        message.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        message.name = [

            'joint1_to_base',
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
        ]

        message.position = [

            math.radians(
                value
            )

            for value in angle_values
        ]

        self.joint_state_pub.publish(
            message
        )

        # Successful feedback clears BOTH kinds of error state.

        if self.invalid_feedback_samples > 0:

            self.get_logger().info(
                'REAL_FEEDBACK_RECOVERED: '
                f'angles={['%.1f' % value for value in angle_values]}'
            )

        self.invalid_feedback_samples = 0

        self.consecutive_feedback_errors = 0

        self.publish_connection(
            'CONNECTED'
        )


    # ========================================================
    # Emergency stop
    # ========================================================

    def perform_stop(self):

        # SAFE_STOP is latched.
        #
        # Repeated stop messages must NOT repeatedly transmit
        # stop() to the physical robot.
        if self.stop_latched:

            return

        self.stop_latched = True

        self.arm_command_pending = False
        self.latest_arm_degrees = None

        try:

            if hasattr(
                self.robot,
                'stop',
            ):

                self.robot.stop()

            elif hasattr(
                self.robot,
                'pause',
            ):

                self.robot.pause()

        except Exception:

            pass

        self.publish_connection(
            'STOPPED'
        )

        self.get_logger().error(
            'REAL ROBOT STOPPED'
        )

    def safety_callback(
        self,
        message,
    ):

        if message.data:

            self.perform_stop()

    def external_estop_callback(
        self,
        message,
    ):

        if message.data:

            self.perform_stop()

            self.publish_fault(
                'EXTERNAL_EMERGENCY_STOP'
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = RealRobotDriver()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
