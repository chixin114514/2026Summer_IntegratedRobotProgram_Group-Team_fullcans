import math

import rclpy

from rclpy.node import Node

from std_msgs.msg import (
    Empty,
    Float64,
)

from joint_tuner import (
    HOME,
    forward_kinematics,
    solve_ik,
)


# ============================================================
# Task geometry
#
# Coordinates are relative to the robot base.
# ============================================================

PICK_X = 0.180
PICK_Y = 0.100

PLACE_X = 0.180
PLACE_Y = -0.100


# The manually calibrated pose showed:
#
# X = 0.176
# Y = 0.113
# Z = 0.038
#
# directly beside the red object.
#
# We therefore use approximately 4 cm as the tool pickup
# height and 11 cm as the safe travel height.

PICK_Z = 0.040

SAFE_Z = 0.110


# Maximum permitted tool-position error when grasping.
GRASP_TOLERANCE = 0.025

# Physical finger travel in metres.
#
# 0.000 = fully open.
# ~0.0055 = around a 50 mm cube.
GRIPPER_OPEN = 0.0000
GRIPPER_CLOSED = 0.0080


# Manually calibrated pose from the tuning panel.
A_CALIBRATION_SEED = [
    math.radians(25.0),
    math.radians(54.0),
    math.radians(-13.0),
    math.radians(33.0),
    math.radians(29.0),
    math.radians(88.0),
]


# B is on the opposite side of the robot.
B_CALIBRATION_SEED = [
    math.radians(-25.0),
    math.radians(54.0),
    math.radians(-13.0),
    math.radians(33.0),
    math.radians(29.0),
    math.radians(88.0),
]


class AutoPickPlace(Node):

    def __init__(self):

        super().__init__(
            'task2_auto_pick_place'
        )

        # -----------------------------------------------------
        # Six arm publishers
        # -----------------------------------------------------

        self.joint_publishers = []

        for index in range(1, 7):

            self.joint_publishers.append(
                self.create_publisher(
                    Float64,
                    f'/task2/joint{index}/cmd_pos',
                    10,
                )
            )

        # -----------------------------------------------------
        # Gripper
        # -----------------------------------------------------

        self.attach_pub = (
            self.create_publisher(
                Empty,
                '/task2/gripper/attach',
                10,
            )
        )

        self.detach_pub = (
            self.create_publisher(
                Empty,
                '/task2/gripper/detach',
                10,
            )
        )

        # Physical two-finger gripper publishers.

        self.left_finger_pub = (
            self.create_publisher(
                Float64,
                '/task2/gripper/left_cmd_pos',
                10,
            )
        )

        self.right_finger_pub = (
            self.create_publisher(
                Float64,
                '/task2/gripper/right_cmd_pos',
                10,
            )
        )

        self.gripper_position = (
            GRIPPER_OPEN
        )


        # -----------------------------------------------------
        # Automatically solve all required Cartesian points.
        # -----------------------------------------------------

        self.get_logger().info(
            'Automatically solving Task 2 IK...'
        )

        self.pose_a_pick = (
            self.find_pose(
                [
                    PICK_X,
                    PICK_Y,
                    PICK_Z,
                ],
                A_CALIBRATION_SEED,
                'A_PICK',
            )
        )

        self.pose_a_high = (
            self.find_pose(
                [
                    PICK_X,
                    PICK_Y,
                    SAFE_Z,
                ],
                self.pose_a_pick,
                'A_APPROACH',
            )
        )

        # -----------------------------------------------------
        # B is geometrically symmetric with A.
        #
        # Do NOT solve B with an independent six-joint IK.
        # A different IK branch can change wrist roll / pitch,
        # causing the grasped cube to arrive at B tilted.
        #
        # Instead:
        #   - preserve J2 ... J6 exactly;
        #   - rotate only J1 around the vertical base axis.
        #
        # This preserves the cube's upright orientation.
        # -----------------------------------------------------

        self.pose_b_pick = (
            self.rotate_pose_to_xy(
                self.pose_a_pick,
                PLACE_X,
                PLACE_Y,
                'B_PLACE',
            )
        )

        self.pose_b_high = (
            self.rotate_pose_to_xy(
                self.pose_a_high,
                PLACE_X,
                PLACE_Y,
                'B_APPROACH',
            )
        )

        self.print_solution(
            'A PICK',
            self.pose_a_pick,
        )

        self.print_solution(
            'A HIGH',
            self.pose_a_high,
        )

        self.print_solution(
            'B PLACE',
            self.pose_b_pick,
        )

        self.print_solution(
            'B HIGH',
            self.pose_b_high,
        )

        # -----------------------------------------------------
        # Motion states
        # -----------------------------------------------------

        self.current_pose = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]

        self.start_pose = list(
            self.current_pose
        )

        self.target_pose = list(
            self.current_pose
        )

        self.motion_start = (
            self.now_seconds()
        )

        self.motion_duration = 1.0

        self.motion_active = False

        self.hold_until = 0.0

        self.state_index = -1

        self.state_started = False

        self.grasped = False

        # The detachable-joint plugin begins attached.
        #
        # We explicitly detach before any arm movement so that
        # the red block remains on the table initially.

        self.initial_detach_count = 0

        self.initialising = True

        # 20 Hz trajectory loop.
        self.timer = self.create_timer(
            0.05,
            self.update,
        )

        self.sequence = [

            (
                'HOME',
                HOME,
                4.0,
                1.0,
            ),

            (
                'A_APPROACH',
                self.pose_a_high,
                4.0,
                1.0,
            ),

            (
                'A_PICK',
                self.pose_a_pick,
                3.0,
                1.0,
            ),

            (
                'CLOSE_GRIPPER',
                None,
                0.0,
                1.2,
            ),

            (
                'GRASP',
                None,
                0.0,
                0.6,
            ),

            (
                'A_LIFT',
                self.pose_a_high,
                3.5,
                1.0,
            ),

            (
                'B_APPROACH',
                self.pose_b_high,
                5.0,
                1.0,
            ),

            (
                'B_PLACE',
                self.pose_b_pick,
                4.5,
                2.0,
            ),

            # Open the physical fingers first while the
            # detachable joint still holds the cube rigidly.
            #
            # The fingers therefore cannot knock the cube over
            # during release.

            (
                'OPEN_GRIPPER',
                None,
                0.0,
                1.5,
            ),

            # Only after both fingers are clear do we release
            # the cube into Gazebo physics.

            (
                'RELEASE',
                None,
                0.0,
                1.5,
            ),

            # Lift away only after the cube has had time
            # to settle flat on the table.

            (
                'B_LIFT',
                self.pose_b_high,
                3.5,
                1.0,
            ),

            (
                'RETURN_HOME',
                HOME,
                4.0,
                2.0,
            ),
        ]

        self.get_logger().info(
            'Automatic pick-and-place controller ready.'
        )

    # ========================================================
    # Helpers
    # ========================================================

    def now_seconds(self):

        return (
            self.get_clock()
            .now()
            .nanoseconds
            /
            1e9
        )

    def find_pose(
        self,
        xyz,
        seed,
        name,
    ):

        # First attempt uses the calibrated seed.
        seeds = [
            list(seed),

            list(HOME),

            [
                seed[0],
                math.radians(40),
                math.radians(-20),
                math.radians(20),
                math.radians(40),
                seed[5],
            ],

            [
                seed[0],
                math.radians(65),
                math.radians(-30),
                math.radians(0),
                math.radians(30),
                seed[5],
            ],
        ]

        best_pose = None

        best_error = float('inf')

        for trial_seed in seeds:

            success, pose, error = (
                solve_ik(
                    xyz,
                    trial_seed,
                )
            )

            if error < best_error:

                best_error = error

                best_pose = list(
                    pose
                )

            if success:

                self.get_logger().info(
                    f'{name}: IK solved, '
                    f'error={error * 1000:.1f} mm'
                )

                return list(
                    pose
                )

        if best_error <= 0.020:

            self.get_logger().warn(
                f'{name}: accepting closest IK '
                f'solution, error='
                f'{best_error * 1000:.1f} mm'
            )

            return best_pose

        raise RuntimeError(
            f'No safe IK solution for {name}. '
            f'Best error = '
            f'{best_error * 1000:.1f} mm'
        )

    def rotate_pose_to_xy(
        self,
        source_pose,
        target_x,
        target_y,
        name,
    ):

        # Current Cartesian position produced by the
        # already-calibrated A-side posture.

        source_xyz = (
            forward_kinematics(
                source_pose
            )
        )

        source_heading = math.atan2(
            source_xyz[1],
            source_xyz[0],
        )

        target_heading = math.atan2(
            target_y,
            target_x,
        )

        result = list(
            source_pose
        )

        # Rotate only the vertical base joint.
        #
        # J2 ... J6 remain unchanged, therefore the tool's
        # roll / pitch and the cube's upright orientation
        # are preserved.

        result[0] += (
            target_heading
            -
            source_heading
        )

        # MechArm J1 physical limit.
        if not (
            -2.792527
            <= result[0]
            <= 2.792527
        ):

            raise RuntimeError(
                f'{name}: required J1 is outside limit.'
            )

        xyz = (
            forward_kinematics(
                result
            )
        )

        xy_error = math.hypot(
            xyz[0] - target_x,
            xyz[1] - target_y,
        )

        self.get_logger().info(
            f'{name}: orientation-preserving '
            f'base rotation, '
            f'XY error={xy_error * 1000:.1f} mm'
        )

        return result

    def print_solution(
        self,
        name,
        q,
    ):

        xyz = (
            forward_kinematics(
                q
            )
        )

        degrees = [
            round(
                math.degrees(v),
                1,
            )
            for v in q
        ]

        self.get_logger().info(
            f'{name}: '
            f'J={degrees} '
            f'XYZ=['
            f'{xyz[0]:.3f}, '
            f'{xyz[1]:.3f}, '
            f'{xyz[2]:.3f}]'
        )

    # ========================================================
    # Robot commands
    # ========================================================

    def publish_pose(
        self,
        pose,
    ):

        for pub, value in zip(
            self.joint_publishers,
            pose,
        ):

            msg = Float64()

            msg.data = float(
                value
            )

            pub.publish(
                msg
            )

    def start_motion(
        self,
        target,
        duration,
    ):

        self.start_pose = list(
            self.current_pose
        )

        self.target_pose = list(
            target
        )

        self.motion_start = (
            self.now_seconds()
        )

        self.motion_duration = max(
            0.2,
            float(duration),
        )

        self.motion_active = True

    def update_motion(self):

        now = self.now_seconds()

        progress = (
            (
                now
                -
                self.motion_start
            )
            /
            self.motion_duration
        )

        progress = max(
            0.0,
            min(
                1.0,
                progress,
            ),
        )

        # Smooth acceleration and deceleration.
        smooth = (
            3.0
            *
            progress
            *
            progress
            -
            2.0
            *
            progress
            *
            progress
            *
            progress
        )

        self.current_pose = [

            self.start_pose[i]
            +
            (
                self.target_pose[i]
                -
                self.start_pose[i]
            )
            *
            smooth

            for i in range(6)
        ]

        self.publish_pose(
            self.current_pose
        )

        if progress >= 1.0:

            self.current_pose = list(
                self.target_pose
            )

            self.motion_active = False

            return True

        return False

    # ========================================================
    # Gripper
    # ========================================================

    def publish_gripper(self):

        msg_left = Float64()
        msg_right = Float64()

        msg_left.data = float(
            self.gripper_position
        )

        msg_right.data = float(
            self.gripper_position
        )

        self.left_finger_pub.publish(
            msg_left
        )

        self.right_finger_pub.publish(
            msg_right
        )

    def set_gripper(
        self,
        position,
    ):

        self.gripper_position = max(
            0.0,
            min(
                0.010,
                float(position),
            ),
        )

        self.publish_gripper()

    def detach_object(self):

        self.detach_pub.publish(
            Empty()
        )

        self.grasped = False

        self.get_logger().info(
            'GRIPPER: detached.'
        )

    def attach_object(self):

        xyz = (
            forward_kinematics(
                self.current_pose
            )
        )

        target = [
            PICK_X,
            PICK_Y,
            PICK_Z,
        ]

        error = math.sqrt(
            sum(
                (
                    xyz[i]
                    -
                    target[i]
                )
                **
                2
                for i in range(3)
            )
        )

        self.get_logger().info(
            f'Grasp position error: '
            f'{error * 1000:.1f} mm'
        )

        if error > GRASP_TOLERANCE:

            raise RuntimeError(
                'Refusing to grasp because '
                'the tool is too far from '
                'the red object.'
            )

        self.attach_pub.publish(
            Empty()
        )

        self.grasped = True

        self.get_logger().info(
            'GRIPPER: target attached.'
        )

    # ========================================================
    # State machine
    # ========================================================

    def update(self):

        # Continuously hold the current physical
        # gripper opening.

        self.publish_gripper()

        # -----------------------------------------------------
        # Initial release of detachable joint.
        # Send it several times while Gazebo plugins initialise.
        # -----------------------------------------------------

        if self.initialising:

            self.detach_pub.publish(
                Empty()
            )

            self.initial_detach_count += 1

            if (
                self.initial_detach_count
                >=
                30
            ):

                self.initialising = False

                self.state_index = 0

                self.state_started = False

                self.get_logger().info(
                    'Initial object release completed.'
                )

            return

        if self.state_index >= len(
            self.sequence
        ):

            return

        (
            name,
            target,
            motion_time,
            hold_time,
        ) = self.sequence[
            self.state_index
        ]

        # -----------------------------------------------------
        # Start state
        # -----------------------------------------------------

        if not self.state_started:

            self.get_logger().info(
                f'STATE: {name}'
            )

            if target is not None:

                self.start_motion(
                    target,
                    motion_time,
                )

            elif name == 'CLOSE_GRIPPER':

                self.set_gripper(
                    GRIPPER_CLOSED
                )

                self.get_logger().info(
                    'GRIPPER: closing physical fingers.'
                )

                self.hold_until = (
                    self.now_seconds()
                    +
                    hold_time
                )

            elif name == 'GRASP':

                self.attach_object()

                self.hold_until = (
                    self.now_seconds()
                    +
                    hold_time
                )

            elif name == 'RELEASE':

                self.detach_object()

                self.hold_until = (
                    self.now_seconds()
                    +
                    hold_time
                )

            elif name == 'OPEN_GRIPPER':

                self.set_gripper(
                    GRIPPER_OPEN
                )

                self.get_logger().info(
                    'GRIPPER: opening physical fingers.'
                )

                self.hold_until = (
                    self.now_seconds()
                    +
                    hold_time
                )

            self.state_started = True

            return

        # -----------------------------------------------------
        # Moving state
        # -----------------------------------------------------

        if target is not None:

            if self.motion_active:

                finished = (
                    self.update_motion()
                )

                if finished:

                    self.hold_until = (
                        self.now_seconds()
                        +
                        hold_time
                    )

                    xyz = (
                        forward_kinematics(
                            self.current_pose
                        )
                    )

                    self.get_logger().info(
                        f'Reached {name}: '
                        f'X={xyz[0]:.3f} '
                        f'Y={xyz[1]:.3f} '
                        f'Z={xyz[2]:.3f}'
                    )

                return

        # -----------------------------------------------------
        # Hold
        # -----------------------------------------------------

        # RELEASE must be reliable.
        #
        # A single detach message may occasionally be missed by
        # the ROS-Gazebo bridge / detachable-joint plugin.
        #
        # While the RELEASE state is active, continuously send
        # detach commands at the controller update rate.
        #
        # This guarantees that the cube is physically released
        # and gravity takes over.

        if name == 'RELEASE':

            self.detach_pub.publish(
                Empty()
            )

        if (
            self.now_seconds()
            <
            self.hold_until
        ):

            return

        # -----------------------------------------------------
        # Next state
        # -----------------------------------------------------

        self.state_index += 1

        self.state_started = False

        if (
            self.state_index
            >=
            len(
                self.sequence
            )
        ):

            self.get_logger().info(
                '================================'
            )

            self.get_logger().info(
                'TASK 2 PICK-AND-PLACE COMPLETED'
            )

            self.get_logger().info(
                '================================'
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = AutoPickPlace()

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
