import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class JointController(Node):

    def __init__(self):

        super().__init__(
            'task2_joint_controller'
        )

        self.publishers = []

        for index in range(1, 7):

            topic = (
                f'/task2/joint{index}/cmd_pos'
            )

            publisher = self.create_publisher(
                Float64,
                topic,
                10,
            )

            self.publishers.append(
                publisher
            )

        # Safe initial home posture.
        #
        # Values are in radians.
        self.home_pose = [
            0.0,
            math.radians(-20.0),
            math.radians(-70.0),
            math.radians(0.0),
            math.radians(90.0),
            math.radians(0.0),
        ]

        self.timer = self.create_timer(
            2.0,
            self.send_home_once,
        )

        self.home_sent = False

        self.get_logger().info(
            'Task 2 six-joint controller started.'
        )

    def send_joint_targets(
        self,
        targets,
    ):

        if len(targets) != 6:

            raise ValueError(
                'Exactly six joint targets are required.'
            )

        for publisher, target in zip(
            self.publishers,
            targets,
        ):

            message = Float64()

            message.data = float(
                target
            )

            publisher.publish(
                message
            )

    def send_home_once(self):

        if self.home_sent:
            return

        self.send_joint_targets(
            self.home_pose
        )

        self.home_sent = True

        self.get_logger().info(
            'Home joint command sent.'
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = JointController()

    rclpy.spin(
        node
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
