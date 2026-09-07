import rclpy

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Float64

from task2_sim.runtime_config import Task2Config


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

        self.sim_pubs = {}
        self.real_pub = None

        if self.config.is_simulation:

            simulation = (
                self.config.communication[
                    'simulation'
                ]
            )

            topic_keys = {
                'left3':
                    'gripper_left3_topic',
                'left2':
                    'gripper_left2_topic',
                'left1':
                    'gripper_left1_topic',
                'right3':
                    'gripper_right3_topic',
                'right2':
                    'gripper_right2_topic',
                'right1':
                    'gripper_right1_topic',
            }

            for joint_name, key in (
                topic_keys.items()
            ):

                self.sim_pubs[
                    joint_name
                ] = (
                    self.create_publisher(
                        Float64,
                        simulation[key],
                        10,
                    )
                )

            task_gripper = (
                self.config.task[
                    'gripper'
                ]
            )

            self.open_position = float(
                task_gripper[
                    'open_position'
                ]
            )

            self.closed_position = float(
                task_gripper[
                    'closed_position'
                ]
            )

            self.sim_open_angle = float(
                task_gripper[
                    'simulation_open_angle_rad'
                ]
            )

            self.sim_closed_angle = float(
                task_gripper[
                    'simulation_closed_angle_rad'
                ]
            )

            self.get_logger().info(
                'GRIPPER BACKEND = '
                'GAZEBO ADAPTIVE LINKAGE'
            )

            self.get_logger().info(
                'Adaptive gripper angles: '
                f'open={self.sim_open_angle:.3f} rad, '
                f'closed={self.sim_closed_angle:.3f} rad'
            )

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
                'REAL MYCOBOT ADAPTIVE GRIPPER AG'
            )

    def command_callback(
        self,
        message,
    ):

        value = float(
            message.data
        )

        if self.config.is_simulation:

            span = max(
                1e-6,
                self.closed_position
                -
                self.open_position,
            )

            closed_fraction = (
                value
                -
                self.open_position
            ) / span

            closed_fraction = max(
                0.0,
                min(
                    1.0,
                    closed_fraction,
                ),
            )

            q = (
                self.sim_open_angle
                +
                closed_fraction
                *
                (
                    self.sim_closed_angle
                    -
                    self.sim_open_angle
                )
            )

            commands = {
                'left3': q,
                'left2': q,
                'left1': -q,
                'right3': -q,
                'right2': -q,
                'right1': q,
            }

            for joint_name, command in (
                commands.items()
            ):

                output = Float64()
                output.data = float(
                    command
                )

                self.sim_pubs[
                    joint_name
                ].publish(
                    output
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
