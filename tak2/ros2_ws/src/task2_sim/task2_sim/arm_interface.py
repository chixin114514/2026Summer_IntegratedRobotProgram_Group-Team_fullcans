import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class ArmInterface(Node):

    def __init__(self):

        super().__init__(
            'task2_arm_interface'
        )

        self.declare_parameter(
            'mode',
            0,
        )

        self.declare_parameter(
            'simulation_joint_topics',
            [
                '/task2/joint1/cmd_pos',
                '/task2/joint2/cmd_pos',
                '/task2/joint3/cmd_pos',
                '/task2/joint4/cmd_pos',
                '/task2/joint5/cmd_pos',
                '/task2/joint6/cmd_pos',
            ],
        )

        self.declare_parameter(
            'real_joint_command_topic',
            '/mecharm270/joint_command',
        )

        self.mode = int(
            self.get_parameter(
                'mode'
            ).value
        )

        if self.mode not in (
            0,
            1,
        ):

            raise RuntimeError(
                'ArmInterface mode must be 0 or 1.'
            )

        self.command_subscriber = (
            self.create_subscription(
                Float64MultiArray,
                '/task2/arm/joint_command',
                self.command_callback,
                10,
            )
        )

        self.sim_publishers = []

        self.real_publisher = None

        if self.mode == 0:

            topics = list(
                self.get_parameter(
                    'simulation_joint_topics'
                ).value
            )

            if len(topics) != 6:

                raise RuntimeError(
                    'Simulation requires six joint topics.'
                )

            for topic in topics:

                self.sim_publishers.append(
                    self.create_publisher(
                        Float64,
                        topic,
                        10,
                    )
                )

            self.get_logger().info(
                'Arm interface mode: SIMULATION'
            )

        else:

            real_topic = (
                self.get_parameter(
                    'real_joint_command_topic'
                ).value
            )

            self.real_publisher = (
                self.create_publisher(
                    Float64MultiArray,
                    real_topic,
                    10,
                )
            )

            self.get_logger().info(
                'Arm interface mode: REAL ROBOT'
            )

    def command_callback(
        self,
        message,
    ):

        if len(
            message.data
        ) != 6:

            self.get_logger().error(
                'Rejected arm command: '
                'exactly six joint values are required.'
            )

            return

        values = [
            float(value)
            for value in message.data
        ]

        if not all(
            math.isfinite(value)
            for value in values
        ):

            self.get_logger().error(
                'Rejected arm command: '
                'non-finite joint value.'
            )

            return

        if self.mode == 0:

            for publisher, value in zip(
                self.sim_publishers,
                values,
            ):

                output = Float64()

                output.data = value

                publisher.publish(
                    output
                )

        else:

            output = (
                Float64MultiArray()
            )

            output.data = values

            self.real_publisher.publish(
                output
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ArmInterface()

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
