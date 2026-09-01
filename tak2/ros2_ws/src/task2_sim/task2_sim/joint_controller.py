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
        # ROS -> Gazebo publishers
        # -----------------------------------------------------

        self.joint_publishers = []

        for index in range(1, 7):

            publisher = self.create_publisher(
                Float64,
                f'/task2/joint{index}/cmd_pos',
                10,
            )

            self.joint_publishers.append(
                publisher
            )

        # -----------------------------------------------------
        # MechArm joint limits
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

        d = math.radians

        # Gazebo initially creates all six joints at zero.
        self.current_pose = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        # -----------------------------------------------------
        # Task waypoints
        #
        # motion_time:
        #     time spent smoothly moving to the pose
        #
        # hold_time:
        #     time spent stationary after reaching the pose
        # -----------------------------------------------------

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
                5.0,
                2.0,
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
                5.0,
                1.5,
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
                1.5,
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
                0.0,
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
                1.5,
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
                6.0,
                1.5,
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
                1.5,
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
                0.0,
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
                1.5,
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
                5.0,
                2.0,
            ),
        ]

        # -----------------------------------------------------
        # Trajectory state
        # -----------------------------------------------------

        self.step_index = -1

        self.start_pose = list(
            self.current_pose
        )

        self.target_pose = list(
            self.current_pose
        )

        self.motion_start_time = 0.0

        self.motion_duration = 0.0

        self.hold_duration = 0.0

        self.hold_start_time = None

        self.state_name = ''

        self.finished = False

        # 20 Hz command update.
        #
        # The robot therefore receives many small position
        # changes instead of one large instantaneous jump.
        self.timer = self.create_timer(
            0.05,
            self.update,
        )

        self.get_logger().info(
            'Task 2 physics-aware motion controller started.'
        )

    # ---------------------------------------------------------
    # Time helper
    # ---------------------------------------------------------

    def now_seconds(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

    # ---------------------------------------------------------
    # Joint-limit safety
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

            if target < lower or target > upper:

                raise ValueError(
                    f'Joint {index} target '
                    f'{target:.3f} rad exceeds '
                    f'[{lower:.3f}, {upper:.3f}]'
                )

    # ---------------------------------------------------------
    # Send one position command
    # ---------------------------------------------------------

    def publish_pose(
        self,
        pose,
    ):

        self.validate_targets(
            pose
        )

        for publisher, value in zip(
            self.joint_publishers,
            pose,
        ):

            msg = Float64()

            msg.data = float(
                value
            )

            publisher.publish(
                msg
            )

    # ---------------------------------------------------------
    # Start one motion stage
    # ---------------------------------------------------------

    def start_next_step(self):

        self.step_index += 1

        if self.step_index >= len(
            self.waypoints
        ):

            self.finished = True

            self.get_logger().info(
                'Task 2 motion sequence completed.'
            )

            return

        (
            self.state_name,
            target,
            motion_time,
            hold_time,
        ) = self.waypoints[
            self.step_index
        ]

        self.validate_targets(
            target
        )

        self.start_pose = list(
            self.current_pose
        )

        self.target_pose = list(
            target
        )

        self.motion_start_time = (
            self.now_seconds()
        )

        self.motion_duration = float(
            motion_time
        )

        self.hold_duration = float(
            hold_time
        )

        self.hold_start_time = None

        self.get_logger().info(
            f'State: {self.state_name}'
        )

    # ---------------------------------------------------------
    # Smooth interpolation
    # ---------------------------------------------------------

    def smooth_interpolate(
        self,
        start,
        target,
        progress,
    ):

        # Cubic smoothstep:
        #
        # s(0) = 0
        # s(1) = 1
        #
        # velocity also approaches zero at the beginning
        # and end of each movement.

        s = (
            3.0 * progress * progress
            -
            2.0
            * progress
            * progress
            * progress
        )

        return [
            a + (b - a) * s
            for a, b in zip(
                start,
                target,
            )
        ]

    # ---------------------------------------------------------
    # Main 20 Hz state machine
    # ---------------------------------------------------------

    def update(self):

        if self.finished:
            return

        if self.step_index < 0:

            self.start_next_step()

            return

        now = self.now_seconds()

        # -----------------------------------------------------
        # States such as GRASP / RELEASE have no arm motion.
        # -----------------------------------------------------

        if self.motion_duration <= 0.0:

            self.current_pose = list(
                self.target_pose
            )

            self.publish_pose(
                self.current_pose
            )

            if self.hold_start_time is None:

                self.hold_start_time = now

                if self.state_name == 'GRASP':

                    self.get_logger().info(
                        'Grasp hold reached.'
                    )

                elif self.state_name == 'RELEASE':

                    self.get_logger().info(
                        'Release hold reached.'
                    )

            if (
                now
                -
                self.hold_start_time
                >=
                self.hold_duration
            ):

                self.start_next_step()

            return

        # -----------------------------------------------------
        # Smooth physical movement
        # -----------------------------------------------------

        elapsed = (
            now
            -
            self.motion_start_time
        )

        progress = (
            elapsed
            /
            self.motion_duration
        )

        if progress < 1.0:

            progress = max(
                0.0,
                progress,
            )

            commanded_pose = (
                self.smooth_interpolate(
                    self.start_pose,
                    self.target_pose,
                    progress,
                )
            )

            self.current_pose = (
                commanded_pose
            )

            self.publish_pose(
                commanded_pose
            )

            return

        # -----------------------------------------------------
        # Target reached
        # -----------------------------------------------------

        self.current_pose = list(
            self.target_pose
        )

        self.publish_pose(
            self.current_pose
        )

        if self.hold_start_time is None:

            self.hold_start_time = now

            self.get_logger().info(
                f'Reached: {self.state_name}'
            )

            return

        if (
            now
            -
            self.hold_start_time
            >=
            self.hold_duration
        ):

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
