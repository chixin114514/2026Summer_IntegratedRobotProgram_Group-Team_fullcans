import math

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import (
    Float64,
    Float64MultiArray,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class ArmInterface(Node):

    def __init__(self):

        super().__init__(
            'task2_arm_interface'
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

        self.mode = self.config.mode

        # -----------------------------------------------------
        # Unified upper-level command
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

        self.sim_publishers = []

        self.real_publisher = None

        # -----------------------------------------------------
        # MODE 0: Gazebo
        # -----------------------------------------------------

        if self.config.is_simulation:

            topics = (
                self.config.communication[
                    'simulation'
                ][
                    'joint_command_topics'
                ]
            )

            if len(topics) != 6:

                raise RuntimeError(
                    'Simulation configuration must '
                    'contain six joint command topics.'
                )

            for topic in topics:

                self.sim_publishers.append(
                    self.create_publisher(
                        Float64,
                        str(topic),
                        10,
                    )
                )

            self.get_logger().info(
                'ARM BACKEND = GAZEBO'
            )

        # -----------------------------------------------------
        # MODE 1: real MechArm adapter
        # -----------------------------------------------------

        else:

            real_topic = (
                self.config.communication[
                    'real'
                ][
                    'joint_command_topic'
                ]
            )

            self.real_publisher = (
                self.create_publisher(
                    Float64MultiArray,
                    str(real_topic),
                    10,
                )
            )

            self.get_logger().info(
                'ARM BACKEND = REAL MECHARM 270'
            )

    def command_callback(
        self,
        message,
    ):

        values = [
            float(value)
            for value in message.data
        ]

        if len(values) != 6:

            self.get_logger().error(
                'Rejected arm command: '
                'six joints required.'
            )

            return

        if not all(
            math.isfinite(value)
            for value in values
        ):

            self.get_logger().error(
                'Rejected arm command: '
                'non-finite value.'
            )

            return

        if self.config.is_simulation:

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
