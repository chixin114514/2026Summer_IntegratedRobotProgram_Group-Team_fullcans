import math

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import (
    Bool,
    Float64,
    Float64MultiArray,
    String,
)

from task2_sim.kinematics import (
    JointLimitError,
    KinematicsError,
    MechArmKinematics,
    NoIKSolutionError,
    UnreachableTargetError,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class TaskManager(Node):

    def __init__(self):

        super().__init__(
            'task2_task_manager'
        )

        # ====================================================
        # Configuration
        # ====================================================

        config_dir = (
            get_package_share_directory(
                'task2_sim'
            )
            + '/config'
        )

        self.config = Task2Config(
            config_dir
        )

        self.kinematics = (
            MechArmKinematics(
                self.config
            )
        )

        common = (
            self.config.communication[
                'common'
            ]
        )

        task = (
            self.config.task
        )

        # ====================================================
        # Communication
        # ====================================================

        self.arm_command_pub = (
            self.create_publisher(
                Float64MultiArray,
                common[
                    'requested_arm_command_topic'
                ],
                10,
            )
        )

        self.gripper_command_pub = (
            self.create_publisher(
                Float64,
                common[
                    'gripper_command_topic'
                ],
                10,
            )
        )

        self.task_state_pub = (
            self.create_publisher(
                String,
                common[
                    'task_state_topic'
                ],
                10,
            )
        )

        self.task_fault_pub = (
            self.create_publisher(
                String,
                common[
                    'task_fault_topic'
                ],
                10,
            )
        )

        self.robot_state_sub = (
            self.create_subscription(
                JointState,
                common[
                    'robot_state_topic'
                ],
                self.robot_state_callback,
                10,
            )
        )

        self.safety_stop_sub = (
            self.create_subscription(
                Bool,
                common[
                    'safety_stop_topic'
                ],
                self.safety_stop_callback,
                10,
            )
        )


        self.task_start_sub = (
            self.create_subscription(
                Bool,
                common[
                    'task_start_topic'
                ],
                self.task_start_callback,
                10,
            )
        )

        # ====================================================
        # Explicit task parameters
        # ====================================================

        self.home = [
            math.radians(
                float(value)
            )
            for value in (
                task[
                    'home'
                ][
                    'joints_deg'
                ]
            )
        ]

        self.point_a = [

            float(
                task[
                    'point_a'
                ][
                    'x'
                ]
            ),

            float(
                task[
                    'point_a'
                ][
                    'y'
                ]
            ),

            float(
                task[
                    'point_a'
                ][
                    'z'
                ]
            ),
        ]

        self.point_b = [

            float(
                task[
                    'point_b'
                ][
                    'x'
                ]
            ),

            float(
                task[
                    'point_b'
                ][
                    'y'
                ]
            ),

            float(
                task[
                    'point_b'
                ][
                    'z'
                ]
            ),
        ]

        self.safe_height = float(
            task[
                'safe_height'
            ][
                'z'
            ]
        )

        motion = (
            task[
                'motion'
            ]
        )

        self.home_duration = float(
            motion[
                'home_duration_s'
            ]
        )

        self.approach_duration = float(
            motion[
                'approach_duration_s'
            ]
        )

        self.descend_duration = float(
            motion[
                'descend_duration_s'
            ]
        )

        self.lift_duration = float(
            motion[
                'lift_duration_s'
            ]
        )

        self.transfer_duration = float(
            motion[
                'transfer_duration_s'
            ]
        )

        gripper = (
            task[
                'gripper'
            ]
        )

        self.gripper_open = float(
            gripper[
                'open_position'
            ]
        )

        self.gripper_closed = float(
            gripper[
                'closed_position'
            ]
        )

        self.gripper_close_duration = float(
            gripper[
                'close_duration_s'
            ]
        )

        self.gripper_open_duration = float(
            gripper[
                'open_duration_s'
            ]
        )

        # ====================================================
        # Runtime state
        # ====================================================

        self.current_joint_state = None

        self.commanded_pose = [
            0.0
            for _ in range(
                6
            )
        ]

        self.safety_stopped = False

        self.task_stopped = False

        self.task_finished = False

        # The node remains READY until experiment_manager sends
        # /task2/task_start = true.
        self.task_running = False

        self.state_index = -1

        self.state_started = False

        # Current trajectory
        self.motion_active = False

        self.motion_start_pose = list(
            self.commanded_pose
        )

        self.motion_target_pose = list(
            self.commanded_pose
        )

        self.motion_start_time = 0.0

        self.motion_duration = 1.0

        self.hold_until = 0.0

        # ====================================================
        # Solve all task waypoints before ANY motion
        #
        # This is important:
        # if B is unreachable, we must find out BEFORE
        # the robot picks up the object.
        # ====================================================

        try:

            self.build_waypoints()

        except KinematicsError as error:

            self.fail_task(
                'KINEMATICS_INITIALISATION_FAILED: '
                +
                str(error)
            )

            return

        except Exception as error:

            self.fail_task(
                'TASK_INITIALISATION_FAILED: '
                +
                str(error)
            )

            return

        # ====================================================
        # State machine
        # ====================================================

        self.sequence = [

            (
                'HOME',
                'motion',
                self.home,
                self.home_duration,
                1.0,
            ),

            (
                'OPEN_GRIPPER_INITIAL',
                'gripper',
                self.gripper_open,
                0.0,
                self.gripper_open_duration,
            ),

            (
                'A_SAFE',
                'motion',
                self.a_safe,
                self.approach_duration,
                1.0,
            ),

            (
                'A_PICK',
                'motion',
                self.a_pick,
                self.descend_duration,
                1.0,
            ),

            (
                'CLOSE_GRIPPER',
                'gripper',
                self.gripper_closed,
                0.0,
                self.gripper_close_duration,
            ),

            (
                'A_LIFT',
                'motion',
                self.a_safe,
                self.lift_duration,
                1.0,
            ),

            (
                'B_SAFE',
                'motion',
                self.b_safe,
                self.transfer_duration,
                1.0,
            ),

            (
                'B_PLACE',
                'motion',
                self.b_place,
                self.descend_duration,
                1.0,
            ),

            (
                'OPEN_GRIPPER',
                'gripper',
                self.gripper_open,
                0.0,
                self.gripper_open_duration,
            ),

            (
                'B_LIFT',
                'motion',
                self.b_safe,
                self.lift_duration,
                1.0,
            ),

            (
                'RETURN_HOME',
                'motion',
                self.home,
                self.home_duration,
                1.0,
            ),
        ]

        # 20 Hz:
        #
        # small incremental commands are required because
        # safety_monitor rejects large instantaneous changes.
        self.timer = (
            self.create_timer(
                0.05,
                self.update,
            )
        )

        self.publish_task_state(
            'READY'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Task 2 manager ready'
        )

        self.get_logger().info(
            f'Mode: '
            f'{self.config.mode_name()}'
        )

        self.get_logger().info(
            'Same task logic will be used '
            'for simulation and real robot.'
        )

        self.get_logger().info(
            '========================================'
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
    # Pre-compute task waypoints
    # ========================================================

    def build_waypoints(self):

        self.kinematics.validate_joints(
            self.home
        )

        # -----------------------------------------------------
        # A pick
        # -----------------------------------------------------

        pick_seed = (
            self.kinematics.pick_seed()
        )

        self.a_pick = (
            self.kinematics.solve_position(
                self.point_a,
                pick_seed,
            )
        )

        # Once the object is grasped, preserve this exact
        # end-effector orientation for ALL transport motions.

        transport_rotation = (
            self.kinematics.forward_rotation(
                self.a_pick
            )
        )

        a_safe_xyz = [

            self.point_a[0],

            self.point_a[1],

            self.safe_height,
        ]

        b_safe_xyz = [

            self.point_b[0],

            self.point_b[1],

            self.safe_height,
        ]

        # -----------------------------------------------------
        # Lift from A while retaining gripper orientation
        # -----------------------------------------------------

        self.a_safe = (
            self.kinematics.solve_pose(
                a_safe_xyz,
                transport_rotation,
                self.a_pick,
            )
        )

        # -----------------------------------------------------
        # Move to B at safe height while retaining orientation
        # -----------------------------------------------------

        self.b_safe = (
            self.kinematics.solve_pose(
                b_safe_xyz,
                transport_rotation,
                self.a_safe,
            )
        )

        # -----------------------------------------------------
        # Descend vertically to B.
        #
        # Same end-effector orientation again.
        # -----------------------------------------------------

        self.b_place = (
            self.kinematics.solve_pose(
                self.point_b,
                transport_rotation,
                self.b_safe,
            )
        )

        # -----------------------------------------------------
        # Final hard validation
        # -----------------------------------------------------

        for name, pose in [

            (
                'HOME',
                self.home,
            ),

            (
                'A_SAFE',
                self.a_safe,
            ),

            (
                'A_PICK',
                self.a_pick,
            ),

            (
                'B_SAFE',
                self.b_safe,
            ),

            (
                'B_PLACE',
                self.b_place,
            ),

        ]:

            self.kinematics.validate_joints(
                pose
            )

            xyz = (
                self.kinematics.forward_position(
                    pose
                )
            )

            self.get_logger().info(
                f'Waypoint {name}: '
                f'X={xyz[0]:.3f} '
                f'Y={xyz[1]:.3f} '
                f'Z={xyz[2]:.3f}'
            )

    # ========================================================
    # Start one complete pick-and-place trial
    # ========================================================

    def task_start_callback(
        self,
        message,
    ):

        if not message.data:
            return

        if self.safety_stopped:

            self.get_logger().error(
                'Cannot start task: SAFE_STOP is active.'
            )

            return

        if self.task_running:

            self.get_logger().warn(
                'Task start ignored: task already running.'
            )

            return

        self.get_logger().info(
            'New pick-and-place trial requested.'
        )

        self.task_stopped = False
        self.task_finished = False
        self.task_running = True

        self.state_index = 0
        self.state_started = False

        self.motion_active = False

        # Start every trial from the current reported posture.
        if self.current_joint_state is not None:

            self.commanded_pose = list(
                self.current_joint_state
            )

        self.publish_task_state(
            'STARTED'
        )

    # ========================================================
    # Robot / safety feedback
    # ========================================================

    def robot_state_callback(
        self,
        message,
    ):

        if len(
            message.position
        ) < 6:

            return

        self.current_joint_state = [
            float(value)
            for value in (
                message.position[
                    :6
                ]
            )
        ]

    def safety_stop_callback(
        self,
        message,
    ):

        if not message.data:

            return

        if self.safety_stopped:

            return

        self.safety_stopped = True

        self.task_stopped = True

        self.motion_active = False

        self.get_logger().error(
            'Task manager received SAFE_STOP.'
        )

        self.publish_task_state(
            'SAFE_STOP'
        )

    # ========================================================
    # Task status
    # ========================================================

    def publish_task_state(
        self,
        state,
    ):

        message = String()

        message.data = str(
            state
        )

        self.task_state_pub.publish(
            message
        )

    def fail_task(
        self,
        reason,
    ):

        if self.task_stopped:

            return

        self.task_stopped = True

        self.motion_active = False

        reason = str(
            reason
        )

        self.get_logger().error(
            reason
        )

        self.publish_task_state(
            'ERROR: '
            +
            reason
        )

        fault = String()

        fault.data = reason

        self.task_fault_pub.publish(
            fault
        )

    # ========================================================
    # Arm commands
    # ========================================================

    def publish_joint_command(
        self,
        pose,
    ):

        if self.task_stopped:

            return

        try:

            self.kinematics.validate_joints(
                pose
            )

        except JointLimitError as error:

            self.fail_task(
                'JOINT_LIMIT: '
                +
                str(error)
            )

            return

        message = (
            Float64MultiArray()
        )

        message.data = [
            float(value)
            for value in pose
        ]

        self.arm_command_pub.publish(
            message
        )

    # ========================================================
    # Gripper commands
    # ========================================================

    def publish_gripper_command(
        self,
        position,
    ):

        if self.task_stopped:

            return

        message = Float64()

        message.data = float(
            position
        )

        self.gripper_command_pub.publish(
            message
        )

    # ========================================================
    # Smooth arm motion
    # ========================================================

    def start_motion(
        self,
        target,
        duration,
    ):

        # Prefer actual / reported state.
        #
        # If feedback has not arrived yet, use the most recent
        # commanded pose.

        if (
            self.current_joint_state
            is not None
        ):

            start_pose = list(
                self.current_joint_state
            )

        else:

            start_pose = list(
                self.commanded_pose
            )

        self.kinematics.validate_joints(
            start_pose
        )

        self.kinematics.validate_joints(
            target
        )

        self.motion_start_pose = (
            start_pose
        )

        self.motion_target_pose = (
            list(
                target
            )
        )

        self.motion_start_time = (
            self.now_seconds()
        )

        self.motion_duration = max(
            0.5,
            float(duration),
        )

        self.motion_active = True

    @staticmethod
    def smoothstep(
        progress,
    ):

        return (
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

    def update_motion(self):

        elapsed = (
            self.now_seconds()
            -
            self.motion_start_time
        )

        progress = (
            elapsed
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

        smooth = (
            self.smoothstep(
                progress
            )
        )

        pose = [

            self.motion_start_pose[index]
            +
            (
                self.motion_target_pose[index]
                -
                self.motion_start_pose[index]
            )
            *
            smooth

            for index in range(
                6
            )
        ]

        self.publish_joint_command(
            pose
        )

        self.commanded_pose = list(
            pose
        )

        if progress >= 1.0:

            self.commanded_pose = list(
                self.motion_target_pose
            )

            self.publish_joint_command(
                self.commanded_pose
            )

            self.motion_active = False

            return True

        return False

    # ========================================================
    # State machine
    # ========================================================

    def start_state(self):

        if (
            self.state_index
            >=
            len(
                self.sequence
            )
        ):

            self.task_finished = True
            self.task_running = False

            self.publish_task_state(
                'COMPLETED'
            )

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'TASK 2 PICK-AND-PLACE COMPLETED'
            )

            self.get_logger().info(
                '========================================'
            )

            return

        (
            name,
            state_type,
            target,
            duration,
            hold_time,
        ) = self.sequence[
            self.state_index
        ]

        self.publish_task_state(
            name
        )

        self.get_logger().info(
            f'STATE: {name}'
        )

        if state_type == 'motion':

            try:

                self.start_motion(
                    target,
                    duration,
                )

            except KinematicsError as error:

                self.fail_task(
                    f'{name}: '
                    +
                    str(error)
                )

                return

        elif state_type == 'gripper':

            self.publish_gripper_command(
                target
            )

            self.hold_until = (
                self.now_seconds()
                +
                hold_time
            )

        else:

            self.fail_task(
                f'UNKNOWN_STATE_TYPE: '
                f'{state_type}'
            )

            return

        self.state_started = True

    def finish_state(self):

        self.state_index += 1

        self.state_started = False

    def update(self):

        if (
            self.task_stopped
            or
            self.task_finished
            or
            not self.task_running
        ):

            return

        if not self.state_started:

            self.start_state()

            return

        (
            name,
            state_type,
            _,
            _,
            hold_time,
        ) = self.sequence[
            self.state_index
        ]

        # -----------------------------------------------------
        # Motion
        # -----------------------------------------------------

        if state_type == 'motion':

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

                    self.get_logger().info(
                        f'Reached: {name}'
                    )

                return

            if (
                self.now_seconds()
                <
                self.hold_until
            ):

                return

            self.finish_state()

            return

        # -----------------------------------------------------
        # Gripper
        #
        # Re-publish throughout the hold period so the
        # prismatic finger controllers continue receiving
        # the desired position.
        # -----------------------------------------------------

        if state_type == 'gripper':

            gripper_target = (
                self.sequence[
                    self.state_index
                ][
                    2
                ]
            )

            self.publish_gripper_command(
                gripper_target
            )

            if (
                self.now_seconds()
                <
                self.hold_until
            ):

                return

            self.finish_state()

            return


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TaskManager()

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
