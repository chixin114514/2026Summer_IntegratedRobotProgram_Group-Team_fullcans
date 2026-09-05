import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import Float64

from task2_sim.runtime_config import (
    Task2Config,
)


class GripperInterface(Node):

    def __init__(self):

        super().__init__(
            'task2_gripper_interface'
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

        self.command_sub = (
            self.create_subscription(
                Float64,
                common[
                    'gripper_command_topic'
                ],
                self.command_callback,
                10,
            )
        )

        self.left_pub = None
        self.right_pub = None
        self.real_pub = None

        # -----------------------------------------------------
        # Simulation physical fingers
        # -----------------------------------------------------

        if self.config.is_simulation:

            simulation = (
                self.config.communication[
                    'simulation'
                ]
            )

            self.left_pub = (
                self.create_publisher(
                    Float64,
                    simulation[
                        'gripper_left_topic'
                    ],
                    10,
                )
            )

            self.right_pub = (
                self.create_publisher(
                    Float64,
                    simulation[
                        'gripper_right_topic'
                    ],
                    10,
                )
            )

            self.get_logger().info(
                'GRIPPER BACKEND = '
                'GAZEBO PHYSICAL FINGERS'
            )

        # -----------------------------------------------------
        # Real gripper adapter
        # -----------------------------------------------------

        else:

            real = (
                self.config.communication[
                    'real'
                ]
            )

            self.real_pub = (
                self.create_publisher(
                    Float64,
                    real[
                        'gripper_command_topic'
                    ],
                    10,
                )
            )

            self.get_logger().info(
                'GRIPPER BACKEND = '
                'REAL MECHARM GRIPPER'
            )

    def command_callback(
        self,
        message,
    ):

        value = float(
            message.data
        )

        if self.config.is_simulation:

            left = Float64()
            right = Float64()

            left.data = value
            right.data = value

            self.left_pub.publish(
                left
            )

            self.right_pub.publish(
                right
            )

        else:

            output = Float64()

            output.data = value

            self.real_pub.publish(
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
