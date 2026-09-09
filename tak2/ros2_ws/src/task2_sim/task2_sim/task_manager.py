import math

import rclpy
from rclpy.executors import ExternalShutdownException

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
    rpy_matrix,
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


        self.software_safety_enabled = bool(
            self.config.safety.get(
                'motion',
                {},
            ).get(
                'software_safety_enabled',
                True,
            )
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


        self.robot_state_source_sub = (
            self.create_subscription(
                String,
                common[
                    'robot_state_source_topic'
                ],
                self.robot_state_source_callback,
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

        self.debug_state_request_sub = None

        if self.config.is_test_mode:

            self.debug_state_request_sub = (
                self.create_subscription(
                    String,
                    common[
                        'debug_state_request_topic'
                    ],
                    self.debug_state_request_callback,
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

        # ====================================================
        # A grasp calibration
        #
        # point_a is the fixed task / object position.
        #
        # grasp_offset_a compensates the real TCP position of
        # the mechArm 270 Pi + myCobot Adaptive Gripper.
        #
        # This keeps the physical A point fixed while allowing
        # millimetre-level gripper calibration.
        # ====================================================

        grasp_offset_a = (
            task.get(
                'grasp_offset_a',
                {},
            )
        )

        self.grasp_offset_a = [

            float(
                grasp_offset_a.get(
                    'x',
                    0.0,
                )
            ),

            float(
                grasp_offset_a.get(
                    'y',
                    0.0,
                )
            ),

            float(
                grasp_offset_a.get(
                    'z',
                    0.0,
                )
            ),
        ]

        self.a_grasp_target = [

            self.point_a[index]
            +
            self.grasp_offset_a[index]

            for index in range(
                3
            )
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

        # ====================================================
        # Independent B placement calibration.
        #
        # IMPORTANT:
        # A grasp calibration is now LOCKED.
        #
        # placement_offset_b only calibrates where the robot
        # releases the already-grasped object.
        # ====================================================

        placement_offset_b = (
            task.get(
                'placement_offset_b',
                {},
            )
        )

        self.placement_offset_b = [

            float(
                placement_offset_b.get(
                    'x',
                    0.0,
                )
            ),

            float(
                placement_offset_b.get(
                    'y',
                    0.0,
                )
            ),

            float(
                placement_offset_b.get(
                    'z',
                    0.0,
                )
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

        self.real_motion_duration_scale = float(
            motion.get(
                'real_motion_duration_scale',
                2.0,
            )
        )


        # ====================================================
        # REAL ROBOT HIGH-LEVEL ACTION QUEUE
        #
        # The task sequence itself is the queue:
        #
        # HOME -> A_SAFE -> A_PREGRASP -> A_PICK -> ...
        #
        # In REAL mode we send exactly ONE target per state.
        # The next state cannot publish until this state's
        # execution window has expired.
        #
        # Simulation keeps the existing smooth trajectory.
        # ====================================================

        real_device = (
            self.config.device.get(
                'real',
                {},
            )
        )

        self.real_queued_execution = bool(
            real_device.get(
                'queued_execution',
                True,
            )
        )

        self.real_motion_settle = max(
            0.0,
            float(
                real_device.get(
                    'motion_settle_s',
                    0.80,
                )
            ),
        )

        self.real_gripper_settle = max(
            0.0,
            float(
                real_device.get(
                    'gripper_settle_s',
                    0.70,
                )
            ),
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

        # ====================================================
        # Online resolved-rate Cartesian servo
        # ====================================================

        self.cartesian_servo_joint_step = (
            math.radians(
                float(
                    motion.get(
                        'cartesian_servo_joint_step_deg',
                        1.0,
                    )
                )
            )
        )

        self.cartesian_servo_position_tolerance = float(
            motion.get(
                'cartesian_servo_position_tolerance_m',
                0.006,
            )
        )

        self.cartesian_servo_orientation_tolerance = (
            math.radians(
                float(
                    motion.get(
                        'cartesian_servo_orientation_tolerance_deg',
                        1.0,
                    )
                )
            )
        )

        self.cartesian_servo_timeout = float(
            motion.get(
                'cartesian_servo_timeout_s',
                3.0,
            )
        )

        self.post_lift_hold = float(
            motion.get(
                'post_lift_hold_s',
                0.80,
            )
        )

        self.a_safe_hold = float(
            motion.get(
                'a_safe_hold_s',
                0.80,
            )
        )

        self.pregrasp_hold = float(
            motion.get(
                'pregrasp_hold_s',
                0.60,
            )
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

        # Gentle adaptive-gripper release.
        #
        # First reduce gripping force / linkage closure,
        # then fully open. This prevents the rotating linkage
        # from suddenly sweeping the object sideways.
        self.gripper_release_partial = (
            self.gripper_open
            +
            0.20
            *
            (
                self.gripper_closed
                -
                self.gripper_open
            )
        )

        # ====================================================
        # Runtime state
        # ====================================================

        self.current_joint_state = None

        self.robot_state_source = 'UNKNOWN'

        # The simulated robot is spawned in HOME.
        # Keep the internal command reference consistent with
        # that physical initial configuration.
        self.commanded_pose = list(
            self.home
        )

        self.safety_stopped = False

        # True only after ALL task waypoints have been
        # successfully calculated and validated.
        #
        # A task-start command received before this point
        # must never move the robot.
        self.initialisation_ok = False

        self.task_stopped = False

        self.task_finished = False

        # Automatic modes remain READY until experiment_manager
        # sends /task2/task_start = true. TEST mode waits for a
        # single-state debug request instead.
        self.task_running = False

        self.state_index = -1

        self.state_started = False

        self.debug_state_active = False

        self.debug_state_name = None

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

        # -----------------------------------------------------
        # Online Cartesian servo runtime.
        # -----------------------------------------------------

        self.cartesian_active = False

        self.cartesian_start_xyz = [
            0.0,
            0.0,
            0.0,
        ]

        self.cartesian_target_xyz = [
            0.0,
            0.0,
            0.0,
        ]

        self.cartesian_start_time = 0.0

        self.cartesian_duration = 1.0

        self.cartesian_deadline = 0.0

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

        # All required Cartesian targets now have valid
        # joint-space solutions.
        self.initialisation_ok = True

        # ====================================================
        # State machine
        # ====================================================

        self.sequence = [

            (
                'HOME',
                'motion',
                self.home,
                self.home_duration,
                0.0,
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
                0.0,
            ),

            (
                'A_PREGRASP',
                'motion',
                self.a_pregrasp,
                0.55,
                0.0,
            ),

            (
                'A_PICK',
                'motion',
                self.a_pick,
                self.descend_duration,
                0.0,
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
                0.0,
            ),

            (
                'B_SAFE',
                'motion',
                self.b_safe,
                self.transfer_duration,
                0.0,
            ),

            (
                'B_PLACE',
                'motion',
                self.b_place,
                self.descend_duration,
                0.0,
            ),

            (
                'RELEASE_GRIPPER_PARTIAL',
                'gripper',
                self.gripper_release_partial,
                0.0,
                0.30,
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
                0.0,
            ),

            (
                'RETURN_HOME',
                'motion',
                self.home,
                self.home_duration,
                0.0,
            ),

        ]

        self.sequence_index_by_name = {
            state[0]: index
            for index, state in enumerate(
                self.sequence
            )
        }

        # 20 Hz:
        #
        # small incremental commands are required because
        # safety_monitor rejects large instantaneous changes.
        self.control_period = (
            0.05
            if self.config.is_simulation
            else 0.10
        )

        self.timer = (
            self.create_timer(
                self.control_period,
                self.update,
            )
        )

        self.get_logger().info(
            'Task control rate: '
            f'{1.0 / self.control_period:.1f} Hz'
        )

        self.ready_announce_count = 0

        self.ready_timer = (
            self.create_timer(
                0.50,
                self.announce_ready,
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


        if (
            self.config.is_real_robot
            and
            self.real_queued_execution
        ):

            self.get_logger().info(
                'REAL EXECUTION = SERIAL ACTION QUEUE'
            )

            self.get_logger().info(
                'One state -> one hardware command -> '
                'wait -> next state'
            )

        self.get_logger().info(
            '========================================'
        )

    # ========================================================
    # READY handshake
    #
    # READY means:
    #   - configuration loaded
    #   - A/B reachable
    #   - IK solved
    #   - joint limits validated
    #
    # experiment_manager is forbidden from starting before it
    # receives this state.
    # ========================================================

    def announce_ready(self):

        if not self.initialisation_ok:
            return

        if self.task_running:
            return

        if self.safety_stopped:
            return

        if self.ready_announce_count >= 10:

            self.ready_timer.cancel()

            return

        self.ready_announce_count += 1

        self.publish_task_state(
            'READY'
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
    # Optional software pose validation
    # ========================================================

    def validate_pose_if_enabled(
        self,
        pose,
    ):

        if not self.software_safety_enabled:

            return

        self.kinematics.validate_joints(
            pose
        )


    # ========================================================
    # Pre-compute task waypoints
    # ========================================================

    def build_waypoints(self):

        # ====================================================
        # STRICT UPRIGHT JOINT MANIFOLD
        #
        # For the current mechArm 270 kinematic chain and the
        # required downward tool orientation:
        #
        #     J4 = 0
        #     J5 = 90 deg - J2 - J3
        #     J6 = J1
        #
        # All stored task poses already obey this relation.
        #
        # Therefore:
        #   - no numerical full-pose IK is required
        #   - no Cartesian servo is required
        #   - no offline path sampling is required
        #
        # Runtime only performs smooth interpolation of
        # J1/J2/J3 and analytically reconstructs J4/J5/J6.
        # ====================================================

        upright = (
            self.config.task[
                'kinematics'
            ][
                'upright_seed_deg'
            ]
        )

        def get_pose(
            name,
        ):

            pose = (
                self.kinematics
                .degrees_to_radians(
                    upright[
                        name
                    ]
                )
            )

            self.validate_pose_if_enabled(
                pose
            )

            return pose


        self.a_pick = get_pose(
            'a_pick'
        )

        self.a_pregrasp = get_pose(
            'a_pregrasp'
        )

        self.a_safe = get_pose(
            'a_safe'
        )

        self.b_safe = get_pose(
            'b_safe'
        )

        self.b_place = get_pose(
            'b_place'
        )


        self.a_pick_xyz = (
            self.kinematics.forward_position(
                self.a_pick
            )
        )

        self.a_pregrasp_xyz = (
            self.kinematics.forward_position(
                self.a_pregrasp
            )
        )

        self.a_safe_xyz = (
            self.kinematics.forward_position(
                self.a_safe
            )
        )

        self.b_safe_xyz = (
            self.kinematics.forward_position(
                self.b_safe
            )
        )

        self.b_place_xyz = (
            self.kinematics.forward_position(
                self.b_place
            )
        )


        self.b_release_target = list(
            self.b_place_xyz
        )


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
                'A_PREGRASP',
                self.a_pregrasp,
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

            self.validate_pose_if_enabled(
                pose
            )

            xyz = (
                self.kinematics
                .forward_position(
                    pose
                )
            )

            self.get_logger().info(
                f'Upright waypoint {name}: '
                f'X={xyz[0]:.3f} '
                f'Y={xyz[1]:.3f} '
                f'Z={xyz[2]:.3f}'
            )


        self.get_logger().info(
            'CALIBRATED WAYPOINT MODE: '
            'all six configured joint angles are respected'
        )

    def task_start_callback(
        self,
        message,
    ):

        if not message.data:
            return

        if self.config.is_test_mode:

            self.get_logger().warn(
                'Task start ignored in TEST_REAL_ROBOT; '
                'use /task2/debug_state_request.'
            )

            return

        if not self.initialisation_ok:

            self.get_logger().error(
                'Task start rejected: '
                'kinematics initialisation is not complete.'
            )

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
        self.cartesian_active = False

        # -----------------------------------------------------
        # Trial-start reference
        #
        # Simulation:
        # the previous trial already ended at commanded HOME.
        # Preserve that continuous command reference. Gazebo
        # feedback may lag slightly and must not create a false
        # command jump at the next trial.
        #
        # Real robot:
        # always begin from measured hardware state.
        # -----------------------------------------------------

        if (
            self.config.is_real_robot
            and
            self.current_joint_state is not None
        ):

            self.commanded_pose = list(
                self.current_joint_state
            )

        self.publish_task_state(
            'STARTED'
        )

    def debug_state_request_callback(
        self,
        message,
    ):

        if not self.config.is_test_mode:

            return

        requested_state = message.data

        if self.safety_stopped:

            self.get_logger().error(
                'Debug state rejected: SAFE_STOP is active.'
            )

            return

        if not self.initialisation_ok:

            self.get_logger().error(
                'Debug state rejected: task initialisation '
                'is not complete.'
            )

            return

        if self.task_stopped:

            self.get_logger().error(
                'Debug state rejected: task is stopped.'
            )

            return

        if (
            self.task_running
            or
            self.state_started
            or
            self.motion_active
            or
            self.cartesian_active
        ):

            self.get_logger().warn(
                'Debug state rejected: another action is running.'
            )

            return

        if requested_state not in self.sequence_index_by_name:

            valid_states = ', '.join(
                self.sequence_index_by_name.keys()
            )

            self.get_logger().error(
                'Debug state rejected: unknown state '
                f'"{requested_state}". Valid states: '
                f'{valid_states}'
            )

            return

        self.task_running = True
        self.task_finished = False
        self.state_index = (
            self.sequence_index_by_name[
                requested_state
            ]
        )
        self.state_started = False
        self.motion_active = False
        self.cartesian_active = False
        self.debug_state_active = True
        self.debug_state_name = requested_state

        self.get_logger().info(
            'DEBUG STATE REQUEST: '
            f'{requested_state}'
        )

    # ========================================================
    # Robot / safety feedback
    # ========================================================

    def robot_state_source_callback(
        self,
        message,
    ):

        self.robot_state_source = (
            message.data.strip()
        )


    def robot_state_callback(
        self,
        message,
    ):

        if len(
            message.position
        ) < 6:

            return

        # Real mode must only use PHYSICAL measured feedback.
        # Never mistake COMMAND_FALLBACK for actual robot pose.
        if (
            self.config.is_real_robot
            and
            self.robot_state_source
            !=
            'MEASURED'
        ):

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
        self.cartesian_active = False

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
        self.cartesian_active = False

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

            self.validate_pose_if_enabled(
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

        # -----------------------------------------------------
        # Motion start source
        #
        # Simulation:
        # keep the commanded trajectory continuous. Gazebo
        # measured joints naturally lag the controller by a
        # small amount; restarting from delayed feedback can
        # create an artificial command jump at state changes.
        #
        # Real robot:
        # always prefer measured hardware state for safety.
        # -----------------------------------------------------

        if self.config.is_simulation:

            start_pose = list(
                self.commanded_pose
            )

        elif (
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

        self.validate_pose_if_enabled(
            start_pose
        )

        self.validate_pose_if_enabled(
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

        effective_duration = float(
            duration
        )

        if self.config.is_real_robot:

            effective_duration *= (
                self.real_motion_duration_scale
            )

        self.motion_duration = max(
            0.5,
            effective_duration,
        )

        self.motion_active = True

    # ========================================================
    # Online resolved-rate Cartesian servo
    #
    # This avoids:
    #   - hundreds of startup IK solves
    #   - IK branch jumps between dense samples
    #   - long startup pauses
    #
    # At every 20 Hz control tick:
    #
    # desired Cartesian pose
    #       ↓
    # pose error
    #       ↓
    # numerical Jacobian
    #       ↓
    # damped least squares
    #       ↓
    # small joint increment
    #
    # The tool rotation is fixed upright for the entire
    # Cartesian movement.
    # ========================================================

    def start_cartesian_motion(
        self,
        target_xyz,
        duration,
    ):

        if len(
            target_xyz
        ) != 3:

            raise KinematicsError(
                'Cartesian target must contain XYZ.'
            )

        if self.config.is_simulation:

            start_pose = list(
                self.commanded_pose
            )

        elif (
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

        self.validate_pose_if_enabled(
            start_pose
        )

        self.commanded_pose = list(
            start_pose
        )

        self.cartesian_start_xyz = (
            self.kinematics.forward_position(
                start_pose
            )
        )

        self.cartesian_target_xyz = [
            float(value)
            for value in target_xyz
        ]

        effective_duration = float(
            duration
        )

        if self.config.is_real_robot:

            effective_duration *= (
                self.real_motion_duration_scale
            )

        self.cartesian_duration = max(
            0.5,
            effective_duration,
        )

        self.cartesian_start_time = (
            self.now_seconds()
        )

        self.cartesian_deadline = (
            self.cartesian_start_time
            +
            self.cartesian_duration
            +
            self.cartesian_servo_timeout
        )

        self.cartesian_active = True


    def update_cartesian_motion(
        self,
    ):

        now = (
            self.now_seconds()
        )

        elapsed = (
            now
            -
            self.cartesian_start_time
        )

        progress = (
            elapsed
            /
            self.cartesian_duration
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

        # -----------------------------------------------------
        # Straight Cartesian reference.
        #
        # Vertical states:
        #   X/Y remain fixed.
        #
        # B_SAFE:
        #   Z remains fixed.
        #
        # The same code handles both automatically.
        # -----------------------------------------------------

        desired_xyz = [

            self.cartesian_start_xyz[index]
            +
            (
                self.cartesian_target_xyz[index]
                -
                self.cartesian_start_xyz[index]
            )
            *
            smooth

            for index in range(
                3
            )
        ]

        joints = list(
            self.commanded_pose
        )

        current_xyz = (
            self.kinematics.forward_position(
                joints
            )
        )

        current_rotation = (
            self.kinematics.forward_rotation(
                joints
            )
        )

        position_error = [

            desired_xyz[index]
            -
            current_xyz[index]

            for index in range(
                3
            )
        ]

        orientation_error = (
            self.kinematics.orientation_error(
                current_rotation,
                self.upright_rotation,
            )
        )

        position_norm = math.sqrt(
            sum(
                value * value
                for value in position_error
            )
        )

        orientation_norm = math.sqrt(
            sum(
                value * value
                for value in orientation_error
            )
        )

        # -----------------------------------------------------
        # Final target reached.
        # -----------------------------------------------------

        if (
            progress >= 1.0
            and
            position_norm
            <=
            self.cartesian_servo_position_tolerance
            and
            orientation_norm
            <=
            self.cartesian_servo_orientation_tolerance
        ):

            self.cartesian_active = False

            return True

        # -----------------------------------------------------
        # Timeout:
        # stop instead of hanging forever.
        # -----------------------------------------------------

        if now >= self.cartesian_deadline:

            self.cartesian_active = False

            self.fail_task(
                'CARTESIAN_SERVO_TIMEOUT: '
                f'position_error='
                f'{position_norm * 1000.0:.1f} mm, '
                f'orientation_error='
                f'{math.degrees(orientation_norm):.2f} deg'
            )

            return False

        try:

            jacobian = (
                self.kinematics.numerical_jacobian(
                    joints,
                    include_orientation=True,
                )
            )

            error = (
                position_error
                +
                orientation_error
            )

            delta = (
                self.kinematics.damped_least_squares(
                    jacobian,
                    error,
                )
            )

        except KinematicsError as error:

            self.cartesian_active = False

            self.fail_task(
                'CARTESIAN_SERVO_FAILED: '
                +
                str(error)
            )

            return False

        # -----------------------------------------------------
        # Hard incremental joint limit.
        #
        # 1 degree per 50 ms = max ~20 deg/s.
        #
        # This is comfortably below the existing
        # real-robot 3-degree command-step safety threshold.
        # -----------------------------------------------------

        maximum_delta = max(
            abs(value)
            for value in delta
        )

        if (
            maximum_delta
            >
            self.cartesian_servo_joint_step
        ):

            scale = (
                self.cartesian_servo_joint_step
                /
                maximum_delta
            )

            delta = [
                value
                *
                scale
                for value in delta
            ]

        next_pose = [

            self.kinematics.clamp_joint(
                joints[index]
                +
                delta[index],
                index,
            )

            for index in range(
                6
            )
        ]

        try:

            self.validate_pose_if_enabled(
                next_pose
            )

        except JointLimitError as error:

            self.cartesian_active = False

            self.fail_task(
                'CARTESIAN_SERVO_JOINT_LIMIT: '
                +
                str(error)
            )

            return False

        self.publish_joint_command(
            next_pose
        )

        self.commanded_pose = list(
            next_pose
        )

        return False


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

        # -----------------------------------------------------
        # SIX-JOINT CALIBRATED INTERPOLATION
        #
        # Respect all six manually calibrated waypoint angles.
        #
        # The previous implementation overwrote J4/J5/J6 on
        # every tick and then snapped back to the configured
        # target on the last tick. That could create a sudden
        # physical jump and made manual J4/J5/J6 tuning partly
        # ineffective.
        # -----------------------------------------------------

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

            # 'pose' is already exactly the interpolated final
            # target at progress == 1.0. Keep the same command
            # instead of replacing it with another branch.
            self.commanded_pose = list(
                pose
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

                # =============================================
                # REAL ROBOT:
                # HIGH-LEVEL SERIAL ACTION QUEUE
                #
                # Exactly ONE target is sent for this state.
                # No intermediate trajectory points are sent.
                # =============================================

                if (
                    self.config.is_real_robot
                    and
                    self.real_queued_execution
                ):

                    self.validate_pose_if_enabled(
                        target
                    )

                    effective_duration = max(
                        0.5,
                        float(duration)
                        *
                        self.real_motion_duration_scale,
                    )

                    self.publish_joint_command(
                        target
                    )

                    self.commanded_pose = list(
                        target
                    )

                    self.motion_active = False

                    self.hold_until = (
                        self.now_seconds()
                        +
                        effective_duration
                        +
                        self.real_motion_settle
                    )

                    self.get_logger().info(
                        'REAL_QUEUE START: '
                        f'{name} '
                        f'execution_window='
                        f'{effective_duration:.2f}s '
                        f'settle='
                        f'{self.real_motion_settle:.2f}s'
                    )

                else:

                    # Simulation keeps the proven smooth
                    # interpolation path.
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

        elif state_type == 'cartesian':

            try:

                self.start_cartesian_motion(
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

            # One command on entry.
            self.publish_gripper_command(
                target
            )

            extra_settle = (
                self.real_gripper_settle
                if (
                    self.config.is_real_robot
                    and
                    self.real_queued_execution
                )
                else
                0.0
            )

            self.hold_until = (
                self.now_seconds()
                +
                hold_time
                +
                extra_settle
            )

            if (
                self.config.is_real_robot
                and
                self.real_queued_execution
            ):

                self.get_logger().info(
                    'REAL_QUEUE START: '
                    f'{name} '
                    f'gripper_window='
                    f'{hold_time:.2f}s '
                    f'settle='
                    f'{extra_settle:.2f}s'
                )

        else:

            self.fail_task(
                f'UNKNOWN_STATE_TYPE: '
                f'{state_type}'
            )

            return

        self.state_started = True

    def finish_state(self):

        if self.debug_state_active:

            completed_state = self.debug_state_name

            self.debug_state_active = False
            self.debug_state_name = None
            self.task_running = False
            self.state_started = False
            self.motion_active = False
            self.cartesian_active = False
            self.state_index = -1

            self.publish_task_state(
                'DEBUG_COMPLETED: '
                f'{completed_state}'
            )

            self.get_logger().info(
                'DEBUG STATE COMPLETED: '
                f'{completed_state}'
            )

            return

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

            # =================================================
            # REAL QUEUED EXECUTION
            #
            # The target was sent exactly once in start_state().
            #
            # Until the action deadline expires:
            #     - no later state can start
            #     - no new arm target is published
            #     - no old trajectory point can accumulate
            # =================================================

            if (
                self.config.is_real_robot
                and
                self.real_queued_execution
            ):

                if (
                    self.now_seconds()
                    <
                    self.hold_until
                ):

                    return

                self.get_logger().info(
                    f'REAL_QUEUE DONE: {name}'
                )

                self.get_logger().info(
                    f'Reached: {name}'
                )

                self.finish_state()

                return


            # =================================================
            # SIMULATION
            # Existing smooth trajectory behaviour.
            # =================================================

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
        # Online Cartesian servo
        # -----------------------------------------------------

        if state_type == 'cartesian':

            if self.cartesian_active:

                finished = (
                    self.update_cartesian_motion()
                )

                if self.task_stopped:

                    return

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

            # =================================================
            # REAL:
            # command was sent once in start_state().
            # Never flood the physical gripper with repeats.
            # =================================================

            if (
                self.config.is_real_robot
                and
                self.real_queued_execution
            ):

                if (
                    self.now_seconds()
                    <
                    self.hold_until
                ):

                    return

                self.get_logger().info(
                    f'REAL_QUEUE DONE: {name}'
                )

                self.finish_state()

                return


            # =================================================
            # SIM:
            # keep publishing the desired position so Gazebo's
            # finger controllers continue holding it.
            # =================================================

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

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()
