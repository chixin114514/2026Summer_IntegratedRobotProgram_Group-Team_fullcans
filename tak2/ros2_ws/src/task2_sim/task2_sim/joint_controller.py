import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64


class JointController(Node):

    def __init__(self):

        super().__init__(
            'task2_joint_controller'
        )

        # -----------------------------------------------------
        # Six Gazebo joint command publishers
        # -----------------------------------------------------

        self.joint_publishers = []

        for index in range(1, 7):

            topic = (
                f'/task2/joint{index}/cmd_pos'
            )

            publisher = self.create_publisher(
                Float64,
                topic,
                10,
            )

            self.joint_publishers.append(
                publisher
            )

        # -----------------------------------------------------
        # Joint limits from the MechArm 270 description
        # -----------------------------------------------------

        self.lower_limits = [
            -2.792527,
            -1.3089,
            -3.0543,
            -2.7052,
            -2.0071,
            -3.14,
        ]

        self.upper_limits = [
            2.792527,
            2.0943,
            1.1344,
            2.7052,
            2.0071,
            3.14,
        ]

        # -----------------------------------------------------
        # Fixed-point pick-and-place joint waypoints
        #
        # The A/B points in the scene are approximately:
        #
        # A: x = 0.18, y = +0.10
        # B: x = 0.18, y = -0.10
        #
        # Therefore joint 1 is mirrored around the robot centre:
        #
        # A side: +29 deg
        # B side: -29 deg
        #
        # The remaining joints define approach, lower and lift
        # postures.
        # -----------------------------------------------------

        d = math.radians

        self.waypoints = [

            (
                'HOME',
                [
                    d(0),
                    d(-20),
                    d(-70),
                    d(0),
                    d(90),
                    d(0),
                ],
                4.0,
            ),

            (
                'APPROACH_A',
                [
                    d(29),
                    d(-10),
                    d(-55),
                    d(0),
                    d(70),
                    d(0),
                ],
                4.0,
            ),

            (
                'PICK_A',
                [
                    d(29),
                    d(12),
                    d(-68),
                    d(0),
                    d(58),
                    d(0),
                ],
                4.0,
            ),

            (
                'GRASP',
                [
                    d(29),
                    d(12),
                    d(-68),
                    d(0),
                    d(58),
                    d(0),
                ],
                2.0,
            ),

            (
                'LIFT_A',
                [
                    d(29),
                    d(-8),
                    d(-50),
                    d(0),
                    d(68),
                    d(0),
                ],
                4.0,
            ),

            (
                'APPROACH_B',
                [
                    d(-29),
                    d(-8),
                    d(-50),
                    d(0),
                    d(68),
                    d(0),
                ],
                4.0,
            ),

            (
                'PLACE_B',
                [
                    d(-29),
                    d(12),
                    d(-68),
                    d(0),
                    d(58),
                    d(0),
                ],
                4.0,
            ),

            (
                'RELEASE',
                [
                    d(-29),
                    d(12),
                    d(-68),
                    d(0),
                    d(58),
                    d(0),
                ],
                2.0,
            ),

            (
                'LIFT_B',
                [
                    d(-29),
                    d(-8),
                    d(-50),
                    d(0),
                    d(68),
                    d(0),
                ],
                4.0,
            ),

            (
                'RETURN_HOME',
                [
                    d(0),
                    d(-20),
                    d(-70),
                    d(0),
                    d(90),
                    d(0),
                ],
                4.0,
            ),
        ]

        # -----------------------------------------------------
        # State machine
        # -----------------------------------------------------

        self.current_step = -1

        self.wait_until = (
            self.get_clock()
            .now()
        )

        # Run the state machine at 10 Hz.
        self.timer = self.create_timer(
            0.1,
            self.run_sequence,
        )

        self.finished = False

        self.get_logger().info(
            'Task 2 pick-and-place sequence started.'
        )

    # ---------------------------------------------------------
    # Safety check
    # ---------------------------------------------------------

    def validate_targets(
        self,
        targets,
    ):

        if len(targets) != 6:

            raise ValueError(
                'Exactly six joint targets are required.'
            )

        for index, (
            target,
            lower,
            upper,
        ) in enumerate(
            zip(
                targets,
                self.lower_limits,
                self.upper_limits,
            ),
            start=1,
        ):

            if not (
                lower
                <= target
                <= upper
            ):

                raise ValueError(
                    f'Joint {index} target '
                    f'{target:.3f} rad is outside '
                    f'[{lower:.3f}, {upper:.3f}]'
                )

    # ---------------------------------------------------------
    # Publish one six-joint posture
    # ---------------------------------------------------------

    def send_joint_targets(
        self,
        targets,
    ):

        self.validate_targets(
            targets
        )

        for publisher, target in zip(
            self.joint_publishers,
            targets,
        ):

            message = Float64()

            message.data = float(
                target
            )

            publisher.publish(
                message
            )

    # ---------------------------------------------------------
    # Start the next state
    # ---------------------------------------------------------

    def start_next_step(self):

        self.current_step += 1

        if (
            self.current_step
            >= len(
                self.waypoints
            )
        ):

            self.finished = True

            self.get_logger().info(
                'Task 2 sequence completed.'
            )

            return

        (
            state_name,
            targets,
            wait_seconds,
        ) = self.waypoints[
            self.current_step
        ]

        self.send_joint_targets(
            targets
        )

        self.get_logger().info(
            f'State: {state_name}'
        )

        # These two states currently represent the future
        # gripper actions. The next development stage will
        # replace them with real gripper open/close commands.

        if state_name == 'GRASP':

            self.get_logger().info(
                'Grasp hold point reached.'
            )

        elif state_name == 'RELEASE':

            self.get_logger().info(
                'Release hold point reached.'
            )

        now = (
            self.get_clock()
            .now()
        )

        self.wait_until = (
            now
            +
            rclpy.duration.Duration(
                seconds=wait_seconds
            )
        )

    # ---------------------------------------------------------
    # Non-blocking state machine
    # ---------------------------------------------------------

    def run_sequence(self):

        if self.finished:
            return

        now = (
            self.get_clock()
            .now()
        )

        if self.current_step < 0:

            self.start_next_step()

            return

        if now >= self.wait_until:

            self.start_next_step()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = JointController()

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
