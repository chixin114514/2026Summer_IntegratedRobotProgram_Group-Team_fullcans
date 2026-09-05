import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from sensor_msgs.msg import (
    JointState,
)

from std_msgs.msg import (
    Float64MultiArray,
    String,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class Task2StatePublisher(Node):

    def __init__(self):

        super().__init__(
            'task2_state_publisher'
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

        common = (
            self.config.communication[
                'common'
            ]
        )

        if self.config.is_simulation:

            backend = (
                self.config.communication[
                    'simulation'
                ]
            )

        else:

            backend = (
                self.config.communication[
                    'real'
                ]
            )

        self.source_joint_state_topic = (
            backend[
                'joint_state_topic'
            ]
        )

        # -----------------------------------------------------
        # Canonical six-joint naming
        # -----------------------------------------------------

        self.joint_names = [

            'joint1_to_base',

            'joint2_to_joint1',

            'joint3_to_joint2',

            'joint4_to_joint3',

            'joint5_to_joint4',

            'joint6_to_joint5',
        ]

        self.commanded_positions = [
            0.0
            for _ in range(6)
        ]

        self.measured_positions = None

        self.last_feedback_time = None

        self.feedback_timeout_s = 0.50

        # -----------------------------------------------------
        # Output
        # -----------------------------------------------------

        self.robot_state_pub = (
            self.create_publisher(
                JointState,
                common[
                    'robot_state_topic'
                ],
                10,
            )
        )

        self.source_pub = (
            self.create_publisher(
                String,
                common[
                    'robot_state_source_topic'
                ],
                10,
            )
        )

        # -----------------------------------------------------
        # Validated commands
        #
        # This gives us a safe fallback if measured joint
        # feedback is temporarily unavailable.
        # -----------------------------------------------------

        self.command_sub = (
            self.create_subscription(
                Float64MultiArray,
                common[
                    'arm_command_topic'
                ],
                self.command_callback,
                10,
            )
        )

        # -----------------------------------------------------
        # Actual joint feedback
        #
        # Simulation:
        #   Gazebo -> /joint_states
        #
        # Real mode:
        #   MechArm driver -> /joint_states
        # -----------------------------------------------------

        self.feedback_sub = (
            self.create_subscription(
                JointState,
                self.source_joint_state_topic,
                self.feedback_callback,
                10,
            )
        )

        # Publish state at 10 Hz.
        self.timer = self.create_timer(
            0.10,
            self.publish_robot_state,
        )

        self.get_logger().info(
            'State publisher started.'
        )

        self.get_logger().info(
            f'Mode: '
            f'{self.config.mode_name()}'
        )

        self.get_logger().info(
            'Feedback topic: '
            f'{self.source_joint_state_topic}'
        )

    # ========================================================
    # Time
    # ========================================================

    def now_seconds(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            /
            1e9
        )

    # ========================================================
    # Command fallback
    # ========================================================

    def command_callback(
        self,
        message,
    ):

        if len(
            message.data
        ) != 6:

            return

        self.commanded_positions = [
            float(value)
            for value in message.data
        ]

    # ========================================================
    # Actual robot / Gazebo feedback
    # ========================================================

    def feedback_callback(
        self,
        message,
    ):

        if len(
            message.position
        ) < 6:

            return

        # Prefer mapping by canonical joint name.

        if (
            len(message.name)
            ==
            len(message.position)
        ):

            lookup = {
                name: position
                for name, position in zip(
                    message.name,
                    message.position,
                )
            }

            if all(
                name in lookup
                for name in self.joint_names
            ):

                self.measured_positions = [
                    float(
                        lookup[name]
                    )
                    for name in self.joint_names
                ]

            else:

                # Driver naming may differ.
                # Fall back to the first six joints.
                self.measured_positions = [
                    float(value)
                    for value in message.position[
                        :6
                    ]
                ]

        else:

            self.measured_positions = [
                float(value)
                for value in message.position[
                    :6
                ]
            ]

        self.last_feedback_time = (
            self.now_seconds()
        )

    # ========================================================
    # Unified state publishing
    # ========================================================

    def publish_robot_state(self):

        now = self.now_seconds()

        feedback_valid = False

        if (
            self.measured_positions
            is not None
            and
            self.last_feedback_time
            is not None
        ):

            feedback_valid = (
                now
                -
                self.last_feedback_time
                <=
                self.feedback_timeout_s
            )

        if feedback_valid:

            positions = list(
                self.measured_positions
            )

            source = 'MEASURED'

        else:

            positions = list(
                self.commanded_positions
            )

            source = (
                'COMMAND_FALLBACK'
            )

        state = JointState()

        state.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        state.name = list(
            self.joint_names
        )

        state.position = positions

        self.robot_state_pub.publish(
            state
        )

        source_message = String()

        source_message.data = (
            source
        )

        self.source_pub.publish(
            source_message
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = Task2StatePublisher()

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
