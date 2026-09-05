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

        self.device = str(
            device[
                'serial_device'
            ]
        )

        self.baudrate = int(
            device[
                'baudrate'
            ]
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

        # -----------------------------------------------------
        # Load pymycobot only in REAL mode.
        # -----------------------------------------------------

        try:

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

        self.get_logger().info(
            'Connecting to MechArm 270: '
            f'{self.device} @ {self.baudrate}'
        )

        try:

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
                10,
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

        self.timer = self.create_timer(
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

        try:

            self.robot.send_angles(
                degrees,
                self.arm_speed,
            )

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

        try:

            angles = (
                self.robot.get_angles()
            )

            if (
                angles is None
                or
                len(angles) != 6
            ):

                raise RuntimeError(
                    'Invalid angle feedback.'
                )

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
                    float(value)
                )
                for value in angles
            ]

            self.joint_state_pub.publish(
                message
            )

            self.consecutive_feedback_errors = 0

            self.publish_connection(
                'CONNECTED'
            )

        except Exception as error:

            self.consecutive_feedback_errors += 1

            self.publish_connection(
                'FEEDBACK_ERROR'
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

    # ========================================================
    # Emergency stop
    # ========================================================

    def perform_stop(self):

        self.stop_latched = True

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
