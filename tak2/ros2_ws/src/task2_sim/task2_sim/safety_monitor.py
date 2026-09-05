import math

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    Float64MultiArray,
    String,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class SafetyMonitor(Node):

    def __init__(self):

        super().__init__(
            'task2_safety_monitor'
        )

        # -----------------------------------------------------
        # Load the SAME configuration used by simulation
        # and the real robot.
        # -----------------------------------------------------

        config_dir = (
            get_package_share_directory(
                'task2_sim'
            )
            + '/config'
        )

        self.config = Task2Config(
            config_dir
        )

        self.mode = (
            self.config.mode
        )

        common = (
            self.config.communication[
                'common'
            ]
        )

        safety = (
            self.config.safety
        )

        # -----------------------------------------------------
        # Joint limits
        # -----------------------------------------------------

        limit_config = (
            safety[
                'joint_limits_deg'
            ]
        )

        self.lower_limits = []
        self.upper_limits = []

        for index in range(
            1,
            7,
        ):

            joint = (
                limit_config[
                    f'joint{index}'
                ]
            )

            self.lower_limits.append(
                math.radians(
                    float(
                        joint['min']
                    )
                )
            )

            self.upper_limits.append(
                math.radians(
                    float(
                        joint['max']
                    )
                )
            )

        if self.config.is_simulation:

            maximum_step_deg = float(
                safety[
                    'motion'
                ][
                    'maximum_joint_step_deg_simulation'
                ]
            )

        else:

            maximum_step_deg = float(
                safety[
                    'motion'
                ][
                    'maximum_joint_step_deg_real'
                ]
            )

        self.maximum_joint_step = (
            math.radians(
                maximum_step_deg
            )
        )

        self.get_logger().info(
            'Maximum command step: '
            f'{maximum_step_deg:.1f} deg'
        )

        # -----------------------------------------------------
        # Internal safety state
        # -----------------------------------------------------

        self.stop_latched = False

        self.stop_reason = ''

        self.previous_command = None

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------

        self.validated_command_pub = (
            self.create_publisher(
                Float64MultiArray,
                common[
                    'arm_command_topic'
                ],
                10,
            )
        )

        self.safety_state_pub = (
            self.create_publisher(
                String,
                common[
                    'safety_state_topic'
                ],
                10,
            )
        )

        self.safety_stop_pub = (
            self.create_publisher(
                Bool,
                common[
                    'safety_stop_topic'
                ],
                10,
            )
        )

        # -----------------------------------------------------
        # Subscribers
        # -----------------------------------------------------

        self.command_sub = (
            self.create_subscription(
                Float64MultiArray,
                common[
                    'requested_arm_command_topic'
                ],
                self.command_callback,
                10,
            )
        )

        self.reset_sub = (
            self.create_subscription(
                Bool,
                common[
                    'safety_reset_topic'
                ],
                self.reset_callback,
                10,
            )
        )


        self.task_fault_sub = (
            self.create_subscription(
                String,
                common[
                    'task_fault_topic'
                ],
                self.task_fault_callback,
                10,
            )
        )


        self.task_state_sub = (
            self.create_subscription(
                String,
                common[
                    'task_state_topic'
                ],
                self.task_state_callback,
                10,
            )
        )

        self.publish_state(
            'OK'
        )

        self.get_logger().info(
            'Safety monitor started.'
        )

        self.get_logger().info(
            f'Mode: '
            f'{self.config.mode_name()}'
        )

    # ========================================================
    # Safety-state publishing
    # ========================================================

    def publish_state(
        self,
        state,
    ):

        state_msg = String()

        state_msg.data = state

        self.safety_state_pub.publish(
            state_msg
        )

        stop_msg = Bool()

        stop_msg.data = (
            self.stop_latched
        )

        self.safety_stop_pub.publish(
            stop_msg
        )

    # ========================================================
    # Latching emergency / safe stop
    # ========================================================

    def trigger_stop(
        self,
        reason,
    ):

        self.stop_latched = True

        self.stop_reason = str(
            reason
        )

        message = (
            'SAFE_STOP: '
            +
            self.stop_reason
        )

        self.get_logger().error(
            message
        )

        self.publish_state(
            message
        )

    # ========================================================
    # Manual reset
    #
    # The stop is LATCHED intentionally:
    # once a dangerous command is detected, later commands
    # cannot silently restart the robot.
    # ========================================================

    def reset_callback(
        self,
        message,
    ):

        if not message.data:

            return

        self.stop_latched = False

        self.stop_reason = ''

        self.previous_command = None

        self.get_logger().warn(
            'Safety stop reset.'
        )

        self.publish_state(
            'OK'
        )

    # ========================================================
    # Fault raised by task-level logic
    #
    # Examples:
    #   UNREACHABLE_TARGET
    #   IK_FAILURE
    #   COMMUNICATION_FAILURE
    # ========================================================

    def task_fault_callback(
        self,
        message,
    ):

        reason = (
            message.data.strip()
        )

        if not reason:

            reason = (
                'UNSPECIFIED_TASK_FAULT'
            )

        self.trigger_stop(
            reason
        )

    # ========================================================
    # Trial boundary synchronisation
    #
    # Reset ONLY the command-step reference. Joint limits,
    # NaN checks and SAFE_STOP behaviour remain fully active.
    # ========================================================

    def task_state_callback(
        self,
        message,
    ):

        state = (
            message.data.strip()
        )

        if state in (
            'READY',
            'STARTED',
            'COMPLETED',
        ):

            self.previous_command = None

    # ========================================================
    # Joint command validation
    # ========================================================

    def command_callback(
        self,
        message,
    ):

        # Once stopped, block ALL future motion commands until
        # a deliberate reset is received.

        if self.stop_latched:

            self.get_logger().warn(
                'Arm command rejected: '
                'safety stop is latched.'
            )

            return

        values = [
            float(value)
            for value in message.data
        ]

        # -----------------------------------------------------
        # Check 1: exactly six joints
        # -----------------------------------------------------

        if len(values) != 6:

            self.trigger_stop(
                'INVALID_COMMAND_LENGTH '
                f'expected=6 actual={len(values)}'
            )

            return

        # -----------------------------------------------------
        # Check 2: NaN / Inf
        # -----------------------------------------------------

        if not all(
            math.isfinite(value)
            for value in values
        ):

            self.trigger_stop(
                'NON_FINITE_JOINT_COMMAND'
            )

            return

        # -----------------------------------------------------
        # Check 3: hard joint limits
        # -----------------------------------------------------

        for index, (
            value,
            lower,
            upper,
        ) in enumerate(
            zip(
                values,
                self.lower_limits,
                self.upper_limits,
            ),
            start=1,
        ):

            if (
                value < lower
                or
                value > upper
            ):

                self.trigger_stop(
                    'JOINT_LIMIT '
                    f'joint={index} '
                    f'value_deg='
                    f'{math.degrees(value):.2f} '
                    f'allowed_deg=['
                    f'{math.degrees(lower):.2f},'
                    f'{math.degrees(upper):.2f}]'
                )

                return

        # -----------------------------------------------------
        # Check 4:
        # reject a large instantaneous joint jump.
        #
        # This protects both simulation and real hardware
        # from dangerous discontinuous commands.
        # -----------------------------------------------------

        if self.previous_command is not None:

            for index, (
                current,
                previous,
            ) in enumerate(
                zip(
                    values,
                    self.previous_command,
                ),
                start=1,
            ):

                delta = abs(
                    current
                    -
                    previous
                )

                if (
                    delta
                    >
                    self.maximum_joint_step
                ):

                    self.trigger_stop(
                        'JOINT_STEP_TOO_LARGE '
                        f'joint={index} '
                        f'delta_deg='
                        f'{math.degrees(delta):.2f} '
                        f'max_deg='
                        f'{math.degrees(self.maximum_joint_step):.2f}'
                    )

                    return

        # -----------------------------------------------------
        # Valid command
        #
        # Only THIS publisher can forward a requested motion
        # command to arm_interface.
        # -----------------------------------------------------

        validated = (
            Float64MultiArray()
        )

        validated.data = values

        self.validated_command_pub.publish(
            validated
        )

        self.previous_command = list(
            values
        )

        self.publish_state(
            'OK'
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = SafetyMonitor()

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
