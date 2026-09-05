import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class GripperInterface(Node):

    def __init__(self):

        super().__init__(
            'task2_gripper_interface'
        )

        self.declare_parameter(
            'mode',
            0,
        )

        self.declare_parameter(
            'simulation_left_topic',
            '/task2/gripper/left_cmd_pos',
        )

        self.declare_parameter(
            'simulation_right_topic',
            '/task2/gripper/right_cmd_pos',
        )

        self.declare_parameter(
            'real_gripper_topic',
            '/mecharm270/gripper_command',
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
                'GripperInterface mode must be 0 or 1.'
            )

        self.command_subscriber = (
            self.create_subscription(
                Float64,
                '/task2/gripper/command',
                self.command_callback,
                10,
            )
        )

        self.left_publisher = None
        self.right_publisher = None
        self.real_publisher = None

        if self.mode == 0:

            self.left_publisher = (
                self.create_publisher(
                    Float64,
                    self.get_parameter(
                        'simulation_left_topic'
                    ).value,
                    10,
                )
            )

            self.right_publisher = (
                self.create_publisher(
                    Float64,
                    self.get_parameter(
                        'simulation_right_topic'
                    ).value,
                    10,
                )
            )

            self.get_logger().info(
                'Gripper interface mode: SIMULATION'
            )

        else:

            self.real_publisher = (
                self.create_publisher(
                    Float64,
                    self.get_parameter(
                        'real_gripper_topic'
                    ).value,
                    10,
                )
            )

            self.get_logger().info(
                'Gripper interface mode: REAL ROBOT'
            )

    def command_callback(
        self,
        message,
    ):

        command = float(
            message.data
        )

        if self.mode == 0:

            left_message = Float64()
            right_message = Float64()

            left_message.data = command
            right_message.data = command

            self.left_publisher.publish(
                left_message
            )

            self.right_publisher.publish(
                right_message
            )

        else:

            output = Float64()

            output.data = command

            self.real_publisher.publish(
                output
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = GripperInterface()

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
