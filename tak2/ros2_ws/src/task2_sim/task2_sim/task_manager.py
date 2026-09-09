import math

import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from task2_sim.kinematics import (
    JointLimitError,
    KinematicsError,
    MechArmKinematics,
)
from task2_sim.real_goal_monitor import (
    JointGoalMonitor,
    radians_to_degrees,
    within_tolerance,
)
from task2_sim.runtime_config import Task2Config


class TaskManager(Node):
    """Formal Task2 state machine for simulation and real hardware.

    Simulation keeps the existing smooth interpolated trajectory behaviour.
    Real queued execution sends one complete joint target and waits for fresh
    measured feedback to confirm arrival before advancing to the next state.
    """

    def __init__(self):
        super().__init__('task2_task_manager')

        config_dir = get_package_share_directory('task2_sim') + '/config'
        self.config = Task2Config(config_dir)
        self.kinematics = MechArmKinematics(self.config)
        self.software_safety_enabled = bool(
            self.config.safety.get('motion', {}).get(
                'software_safety_enabled', True
            )
        )

        common = self.config.communication['common']
        task = self.config.task
        motion = task['motion']
        real_device = self.config.device.get('real', {})

        self.arm_command_pub = self.create_publisher(
            Float64MultiArray,
            common['requested_arm_command_topic'],
            10,
        )
        self.gripper_command_pub = self.create_publisher(
            Float64,
            common['gripper_command_topic'],
            10,
        )
        self.task_state_pub = self.create_publisher(
            String,
            common['task_state_topic'],
            10,
        )
        self.task_fault_pub = self.create_publisher(
            String,
            common['task_fault_topic'],
            10,
        )

        self.robot_state_sub = self.create_subscription(
            JointState,
            common['robot_state_topic'],
            self.robot_state_callback,
            10,
        )
        self.robot_state_source_sub = self.create_subscription(
            String,
            common['robot_state_source_topic'],
            self.robot_state_source_callback,
            10,
        )
        self.safety_stop_sub = self.create_subscription(
            Bool,
            common['safety_stop_topic'],
            self.safety_stop_callback,
            10,
        )
        self.safety_reset_sub = self.create_subscription(
            Bool,
            common['safety_reset_topic'],
            self.safety_reset_callback,
            10,
        )
        self.task_fault_sub = self.create_subscription(
            String,
            common['task_fault_topic'],
            self.task_fault_callback,
            10,
        )
        self.task_start_sub = self.create_subscription(
            Bool,
            common['task_start_topic'],
            self.task_start_callback,
            10,
        )

        # Single source of truth for HOME.
        self.home = [
            math.radians(float(value))
            for value in task['home']['joints_deg']
        ]

        self.home_duration = float(motion['home_duration_s'])
        self.approach_duration = float(motion['approach_duration_s'])
        self.descend_duration = float(motion['descend_duration_s'])
        self.lift_duration = float(motion['lift_duration_s'])
        self.transfer_duration = float(motion['transfer_duration_s'])

        # -----------------------------------------------------
        # Gazebo measured-state convergence.
        #
        # Interpolation time only determines the reference
        # trajectory. A simulation motion state is complete
        # only when Gazebo's MEASURED joints physically reach
        # the target and remain there for a short stable time.
        # -----------------------------------------------------

        self.sim_joint_goal_tolerance = math.radians(
            float(
                motion.get(
                    'simulation_joint_goal_tolerance_deg',
                    1.0,
                )
            )
        )

        self.sim_goal_stable_s = max(
            0.0,
            float(
                motion.get(
                    'simulation_goal_stable_s',
                    0.20,
                )
            ),
        )

        self.sim_motion_timeout_s = max(
            0.5,
            float(
                motion.get(
                    'simulation_motion_timeout_s',
                    4.0,
                )
            ),
        )

        self.sim_feedback_max_age_s = max(
            0.05,
            float(
                motion.get(
                    'simulation_feedback_max_age_s',
                    0.50,
                )
            ),
        )

        self.sim_goal_log_period_s = max(
            0.1,
            float(
                motion.get(
                    'simulation_goal_log_period_s',
                    0.50,
                )
            ),
        )

        self.sim_timeout_acceptance = math.radians(
            float(
                motion.get(
                    'simulation_timeout_acceptance_deg',
                    2.0,
                )
            )
        )

        gripper = task['gripper']
        self.gripper_open = float(gripper['open_position'])
        self.gripper_closed = float(gripper['closed_position'])
        self.gripper_close_duration = float(gripper['close_duration_s'])
        self.gripper_open_duration = float(gripper['open_duration_s'])
        self.gripper_release_partial = (
            self.gripper_open
            + 0.20 * (self.gripper_closed - self.gripper_open)
        )

        # Formal real mode is deliberately one-goal-at-a-time. Streaming mode
        # is not part of the acceptance control chain.
        self.real_queued_execution = bool(
            real_device.get('queued_execution', True)
        )
        if self.config.is_real_robot and not self.real_queued_execution:
            raise RuntimeError(
                'Formal real mode requires real.queued_execution=true. '
                'Streaming mode must use a separate fresh_mode=1 design.'
            )

        self.real_gripper_settle = max(
            0.0,
            float(real_device.get('gripper_settle_s', 0.50)),
        )
        self.real_joint_goal_tolerance = math.radians(
            float(real_device.get('real_joint_goal_tolerance_deg', 2.0))
        )
        self.real_home_tolerance = math.radians(
            float(real_device.get('real_home_tolerance_deg', 1.5))
        )
        self.real_goal_stable_s = max(
            0.0,
            float(real_device.get('real_goal_stable_s', 0.50)),
        )
        self.real_motion_timeout_s = max(
            0.5,
            float(real_device.get('real_motion_timeout_s', 20.0)),
        )
        self.real_feedback_max_age_s = max(
            0.05,
            float(real_device.get('real_feedback_max_age_s', 0.75)),
        )
        self.real_goal_log_period_s = max(
            0.1,
            float(real_device.get('real_goal_log_period_s', 1.0)),
        )

        self.current_joint_state = None
        self.last_measured_state_time = None
        self.robot_state_source = 'UNKNOWN'
        self.commanded_pose = list(self.home)

        self.initialisation_ok = False
        self.safety_stopped = False
        self.task_stopped = False
        self.task_finished = False
        self.task_running = False
        self.return_home_confirmed = False

        self.state_index = -1
        self.state_started = False
        self.hold_until = 0.0

        # Simulation motion state.
        self.motion_active = False
        self.motion_start_pose = list(self.commanded_pose)
        self.motion_target_pose = list(self.commanded_pose)
        self.motion_start_time = 0.0
        self.motion_duration = 1.0

        self.sim_goal_stable_since = None
        self.sim_motion_deadline = 0.0
        self.sim_goal_last_log_time = 0.0

        # Real feedback-closed motion state.
        self.real_goal_monitor = None
        self.real_goal_command_sent = False
        self.real_goal_last_log_time = 0.0

        try:
            self.build_waypoints()
        except Exception as error:
            self.fail_task('TASK_INITIALISATION_FAILED: ' + str(error))
            return

        self.initialisation_ok = True

        self.sequence = [
            ('HOME', 'motion', self.home, self.home_duration, 0.0),
            (
                'OPEN_GRIPPER_INITIAL',
                'gripper',
                self.gripper_open,
                0.0,
                self.gripper_open_duration,
            ),
            ('A_SAFE', 'motion', self.a_safe, self.approach_duration, 0.0),
            ('A_PREGRASP', 'motion', self.a_pregrasp, 0.55, 0.0),
            ('A_PICK', 'motion', self.a_pick, self.descend_duration, 0.0),
            (
                'CLOSE_GRIPPER',
                'gripper',
                self.gripper_closed,
                0.0,
                self.gripper_close_duration,
            ),
            ('A_LIFT', 'motion', self.a_safe, self.lift_duration, 0.0),
            ('B_SAFE', 'motion', self.b_safe, self.transfer_duration, 0.0),
            ('B_PLACE', 'motion', self.b_place, self.descend_duration, 0.0),
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
            ('B_LIFT', 'motion', self.b_safe, self.lift_duration, 0.0),
            ('RETURN_HOME', 'motion', self.home, self.home_duration, 0.0),
        ]

        self.control_period = 0.05 if self.config.is_simulation else 0.10
        self.timer = self.create_timer(self.control_period, self.update)
        self.ready_timer = self.create_timer(0.50, self.announce_ready)

        self.get_logger().info(
            f'Task control rate: {1.0 / self.control_period:.1f} Hz'
        )
        self.get_logger().info('========================================')
        self.get_logger().info('Task 2 manager ready')
        self.get_logger().info(f'Mode: {self.config.mode_name()}')
        if self.config.is_real_robot:
            self.get_logger().info(
                'REAL EXECUTION = ONE TARGET + MEASURED FEEDBACK CLOSED LOOP'
            )
        self.get_logger().info('========================================')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def measured_feedback_fresh(self, now=None):
        if self.config.is_simulation:
            return True
        if self.current_joint_state is None or self.last_measured_state_time is None:
            return False
        if now is None:
            now = self.now_seconds()
        return (
            now - self.last_measured_state_time
            <= self.real_feedback_max_age_s
        )

    def simulation_feedback_fresh(
        self,
        now=None,
    ):
        if self.current_joint_state is None:
            return False

        if self.last_measured_state_time is None:
            return False

        if now is None:
            now = self.now_seconds()

        return (
            now
            -
            self.last_measured_state_time
            <=
            self.sim_feedback_max_age_s
        )

    def announce_ready(self):
        if not self.initialisation_ok:
            return
        if self.task_running or self.safety_stopped or self.task_stopped:
            return
        if self.config.is_real_robot and not self.measured_feedback_fresh():
            return
        self.publish_task_state('READY')

    def validate_pose_if_enabled(self, pose):
        if self.software_safety_enabled:
            self.kinematics.validate_joints(pose)

    def build_waypoints(self):
        upright = self.config.task['kinematics']['upright_seed_deg']

        def get_pose(name):
            pose = self.kinematics.degrees_to_radians(upright[name])
            self.validate_pose_if_enabled(pose)
            return pose

        self.a_pick = get_pose('a_pick')
        self.a_pregrasp = get_pose('a_pregrasp')
        self.a_safe = get_pose('a_safe')
        self.b_safe = get_pose('b_safe')
        self.b_place = get_pose('b_place')
        self.validate_pose_if_enabled(self.home)

        for name, pose in [
            ('HOME', self.home),
            ('A_SAFE', self.a_safe),
            ('A_PREGRASP', self.a_pregrasp),
            ('A_PICK', self.a_pick),
            ('B_SAFE', self.b_safe),
            ('B_PLACE', self.b_place),
        ]:
            xyz = self.kinematics.forward_position(pose)
            self.get_logger().info(
                f'Upright waypoint {name}: '
                f'X={xyz[0]:.3f} Y={xyz[1]:.3f} Z={xyz[2]:.3f}'
            )

        self.get_logger().info(
            'CALIBRATED WAYPOINT MODE: all six configured joint angles '
            'are respected; A/B values were not recalibrated.'
        )

    def task_start_callback(self, message):
        if not message.data:
            return
        if not self.initialisation_ok:
            self.get_logger().error('Task start rejected: initialisation incomplete.')
            return
        if self.safety_stopped or self.task_stopped:
            self.get_logger().error('Task start rejected: stop/fault is latched.')
            return
        if self.task_running:
            self.get_logger().warn('Task start ignored: task already running.')
            return
        if self.config.is_real_robot and not self.measured_feedback_fresh():
            self.fail_task(
                'REAL_FEEDBACK_STALE state=TRIAL_START '
                'fresh MEASURED joint feedback is required'
            )
            return

        self.get_logger().info('New pick-and-place trial requested.')
        self.task_stopped = False
        self.task_finished = False
        self.task_running = True
        self.return_home_confirmed = False
        self.state_index = 0
        self.state_started = False
        self.motion_active = False
        self.real_goal_monitor = None
        self.real_goal_command_sent = False

        if self.config.is_real_robot:
            self.commanded_pose = list(
                self.current_joint_state
            )

        elif (
            self.config.is_simulation
            and
            self.current_joint_state is not None
        ):
            self.commanded_pose = list(
                self.current_joint_state
            )

            self.get_logger().info(
                'SIM_TRIAL_RESYNC measured_deg='
                +
                self.format_degrees(
                    self.current_joint_state
                )
            )

        self.publish_task_state('STARTED')

    def robot_state_source_callback(self, message):
        self.robot_state_source = message.data.strip()

    def robot_state_callback(self, message):
        if len(message.position) < 6:
            return

        source = (
            message.header.frame_id.strip()
            or
            self.robot_state_source
        )

        # COMMAND_FALLBACK is useful for display, but must not
        # be treated as physical Gazebo/MechArm convergence.
        if source != 'MEASURED':
            return

        self.current_joint_state = [
            float(value)
            for value in message.position[:6]
        ]

        if self.config.is_real_robot:
            stamp = message.header.stamp

            measured_time = (
                float(stamp.sec)
                +
                float(stamp.nanosec)
                /
                1e9
            )

            self.last_measured_state_time = (
                measured_time
                if measured_time > 0.0
                else self.now_seconds()
            )

        else:
            # For Gazebo, receipt time is sufficient and avoids
            # mixing simulation-clock stamps with wall time.
            self.last_measured_state_time = (
                self.now_seconds()
            )

    def safety_stop_callback(self, message):
        if not message.data or self.safety_stopped:
            return
        self.safety_stopped = True
        self.task_stopped = True
        self.task_running = False
        self.motion_active = False
        self.real_goal_monitor = None
        self.get_logger().error('Task manager received SAFE_STOP.')
        self.publish_task_state('SAFE_STOP')

    def safety_reset_callback(self, message):
        if not message.data:
            return
        if self.task_running:
            self.get_logger().error('Safety reset rejected while task is running.')
            return

        self.safety_stopped = False
        self.task_stopped = False
        self.task_finished = False
        self.state_index = -1
        self.state_started = False
        self.motion_active = False
        self.real_goal_monitor = None
        self.real_goal_command_sent = False
        self.return_home_confirmed = False
        self.get_logger().warn(
            'Task manager safety latch reset. Acceptance manager remains '
            'aborted and must be relaunched after a SAFE_STOP.'
        )
        self.publish_task_state('READY')

    def task_fault_callback(self, message):
        reason = message.data.strip()
        if not reason or self.task_stopped:
            return

        self.task_stopped = True
        self.task_running = False
        self.motion_active = False
        self.real_goal_monitor = None
        self.get_logger().error('Task manager received fault: ' + reason)
        self.publish_task_state('ERROR: ' + reason)

    def publish_task_state(self, state):
        message = String()
        message.data = str(state)
        self.task_state_pub.publish(message)

    def fail_task(self, reason):
        if self.task_stopped:
            return

        reason = str(reason)
        self.task_stopped = True
        self.task_running = False
        self.motion_active = False
        self.real_goal_monitor = None
        self.get_logger().error(reason)
        self.publish_task_state('ERROR: ' + reason)

        fault = String()
        fault.data = reason
        self.task_fault_pub.publish(fault)

    def publish_joint_command(self, pose):
        if self.task_stopped:
            return
        try:
            self.validate_pose_if_enabled(pose)
        except JointLimitError as error:
            self.fail_task('JOINT_LIMIT: ' + str(error))
            return

        message = Float64MultiArray()
        message.data = [float(value) for value in pose]
        self.arm_command_pub.publish(message)

    def publish_gripper_command(self, position):
        if self.task_stopped:
            return
        message = Float64()
        message.data = float(position)
        self.gripper_command_pub.publish(message)

    # ------------------------------------------------------------------
    # Simulation motion. This intentionally preserves the existing
    # smoothstep interpolation path and 20 Hz publication behaviour.
    # ------------------------------------------------------------------
    def start_motion(self, target, duration):

        # -----------------------------------------------------
        # SIMULATION PHYSICAL RESYNC
        #
        # Never inherit the previous ideal command as the
        # physical starting pose. Use Gazebo's latest measured
        # joints at every motion-state boundary.
        #
        # This prevents residual controller tracking error from
        # accumulating from state to state and trial to trial.
        # -----------------------------------------------------

        if (
            self.config.is_simulation
            and
            self.current_joint_state is not None
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

        self.motion_start_pose = start_pose
        self.motion_target_pose = list(
            target
        )

        self.motion_start_time = (
            self.now_seconds()
        )

        self.motion_duration = max(
            0.5,
            float(duration),
        )

        self.sim_goal_stable_since = None

        self.sim_motion_deadline = (
            self.motion_start_time
            +
            self.motion_duration
            +
            self.sim_motion_timeout_s
        )

        self.sim_goal_last_log_time = 0.0

        self.motion_active = True


    @staticmethod
    def smoothstep(progress):
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

        now = self.now_seconds()

        elapsed = (
            now
            -
            self.motion_start_time
        )

        progress = max(
            0.0,
            min(
                1.0,
                elapsed
                /
                self.motion_duration,
            ),
        )

        smooth = self.smoothstep(
            progress
        )

        pose = [
            start
            +
            (
                target
                -
                start
            )
            *
            smooth

            for start, target in zip(
                self.motion_start_pose,
                self.motion_target_pose,
            )
        ]

        # At the end of interpolation always hold the exact
        # calibrated target while Gazebo physically converges.
        if progress >= 1.0:
            pose = list(
                self.motion_target_pose
            )

        self.publish_joint_command(
            pose
        )

        self.commanded_pose = list(
            pose
        )

        if progress < 1.0:
            return False

        state_name = (
            self.sequence[
                self.state_index
            ][0]
            if (
                0
                <=
                self.state_index
                <
                len(self.sequence)
            )
            else
            'UNKNOWN'
        )

        # -----------------------------------------------------
        # Do NOT close the gripper / enter the next state until
        # Gazebo says the joints physically reached the target.
        # -----------------------------------------------------

        if not self.simulation_feedback_fresh(
            now
        ):

            self.sim_goal_stable_since = None

            if now >= self.sim_motion_deadline:

                self.motion_active = False

                self.fail_task(
                    'SIM_FEEDBACK_TIMEOUT '
                    f'state={state_name}'
                )

            return False

        errors = [
            abs(
                current
                -
                target
            )

            for current, target in zip(
                self.current_joint_state,
                self.motion_target_pose,
            )
        ]

        max_error = max(
            errors
        )

        if (
            max_error
            <=
            self.sim_joint_goal_tolerance
        ):

            if self.sim_goal_stable_since is None:
                self.sim_goal_stable_since = now

            stable_s = (
                now
                -
                self.sim_goal_stable_since
            )

            if (
                stable_s
                >=
                self.sim_goal_stable_s
            ):

                self.motion_active = False

                self.commanded_pose = list(
                    self.motion_target_pose
                )

                self.get_logger().info(
                    'SIM_GOAL_REACHED '
                    f'state={state_name} '
                    f'max_error_deg='
                    f'{math.degrees(max_error):.3f} '
                    f'stable_s={stable_s:.2f}'
                )

                return True

        else:

            self.sim_goal_stable_since = None
            stable_s = 0.0

        if now >= self.sim_motion_deadline:

            # -------------------------------------------------
            # Gazebo-only bounded soft acceptance.
            #
            # Small residual PID error under gravity must not
            # abort the complete 5-trial acceptance run.
            #
            # We still keep measured-state resynchronisation at
            # every following state, so this does NOT restore
            # the old cumulative-drift behaviour.
            # -------------------------------------------------

            if (
                max_error
                <=
                self.sim_timeout_acceptance
            ):

                self.motion_active = False

                # IMPORTANT:
                # Preserve the ACTUAL measured Gazebo pose as
                # the next state's reference instead of lying
                # that the exact target was reached.
                self.commanded_pose = list(
                    self.current_joint_state
                )

                self.get_logger().warn(
                    'SIM_GOAL_ACCEPTED '
                    f'state={state_name} '
                    f'max_error_deg='
                    f'{math.degrees(max_error):.3f} '
                    'reason=bounded_gazebo_residual'
                )

                return True

            self.motion_active = False

            self.fail_task(
                'SIM_GOAL_TIMEOUT '
                f'state={state_name} '
                f'target_deg='
                f'{self.format_degrees(self.motion_target_pose)} '
                f'current_deg='
                f'{self.format_degrees(self.current_joint_state)} '
                f'max_error_deg='
                f'{math.degrees(max_error):.3f}'
            )

            return False

        if (
            now
            -
            self.sim_goal_last_log_time
            >=
            self.sim_goal_log_period_s
        ):

            self.sim_goal_last_log_time = now

            self.get_logger().info(
                'SIM_GOAL_WAIT '
                f'state={state_name} '
                f'max_error_deg='
                f'{math.degrees(max_error):.3f}'
            )

        return False

    # ------------------------------------------------------------------
    # Real motion feedback closure.
    # ------------------------------------------------------------------
    def goal_tolerance_for_state(self, name):
        if name in ('HOME', 'RETURN_HOME'):
            return self.real_home_tolerance
        return self.real_joint_goal_tolerance

    def start_real_goal(self, name, target):
        now = self.now_seconds()
        tolerance = self.goal_tolerance_for_state(name)
        self.real_goal_monitor = JointGoalMonitor(
            state_name=name,
            target=target,
            tolerance_rad=tolerance,
            stable_required_s=self.real_goal_stable_s,
            timeout_s=self.real_motion_timeout_s,
            feedback_max_age_s=self.real_feedback_max_age_s,
            started_at=now,
        )
        self.real_goal_command_sent = False
        self.real_goal_last_log_time = 0.0

        # HOME is idempotent: if a fresh measured pose is already within the
        # strict HOME tolerance, do not add another identical command to the
        # hardware queue. Stable verification still has to complete.
        already_at_goal = (
            self.measured_feedback_fresh(now)
            and within_tolerance(
                self.current_joint_state,
                target,
                tolerance,
            )
        )

        if already_at_goal:
            self.get_logger().info(
                f'REAL_GOAL_SKIP state={name} reason=already_within_tolerance '
                f'current_deg={self.format_degrees(self.current_joint_state)}'
            )
        elif self.measured_feedback_fresh(now):
            self.send_real_goal(name, target)
        else:
            self.get_logger().warn(
                f'REAL_GOAL_WAIT state={name} waiting_for=fresh_MEASURED_feedback'
            )

    def send_real_goal(self, name, target):
        if self.real_goal_command_sent:
            return
        self.publish_joint_command(target)
        if self.task_stopped:
            return
        self.commanded_pose = list(target)
        self.real_goal_command_sent = True
        self.get_logger().info(
            f'REAL_GOAL_SENT state={name} '
            f'target_deg={self.format_degrees(target)}'
        )

    @staticmethod
    def format_degrees(values):
        if values is None:
            return 'N/A'
        return '[' + ', '.join(f'{value:.2f}' for value in radians_to_degrees(values)) + ']'

    @staticmethod
    def format_errors_deg(errors):
        if not errors:
            return 'N/A'
        return '[' + ', '.join(
            f'{math.degrees(value):.2f}' for value in errors
        ) + ']'

    def real_goal_failure_text(self, prefix, name, target, evaluation):
        current = self.current_joint_state
        max_error = (
            'N/A'
            if evaluation.max_error_rad is None
            else f'{math.degrees(evaluation.max_error_rad):.2f}'
        )
        age = (
            'N/A'
            if evaluation.feedback_age_s is None
            else f'{evaluation.feedback_age_s:.3f}'
        )
        return (
            f'{prefix} state={name} '
            f'target_deg={self.format_degrees(target)} '
            f'current_deg={self.format_degrees(current)} '
            f'errors_deg={self.format_errors_deg(evaluation.errors_rad)} '
            f'max_error_deg={max_error} feedback_age_s={age}'
        )

    def update_real_goal(self, name, target):
        now = self.now_seconds()
        evaluation = self.real_goal_monitor.evaluate(
            now,
            self.current_joint_state,
            self.last_measured_state_time,
        )

        if evaluation.status == JointGoalMonitor.FEEDBACK_STALE:
            self.fail_task(
                self.real_goal_failure_text(
                    'REAL_FEEDBACK_STALE',
                    name,
                    target,
                    evaluation,
                )
            )
            return

        if evaluation.status == JointGoalMonitor.TIMEOUT:
            text = self.real_goal_failure_text(
                'REAL_GOAL_TIMEOUT',
                name,
                target,
                evaluation,
            )
            self.get_logger().error(text)
            self.fail_task(text)
            return

        if (
            not self.real_goal_command_sent
            and evaluation.status != JointGoalMonitor.WAIT_FEEDBACK
            and evaluation.max_error_rad is not None
            and evaluation.max_error_rad > self.goal_tolerance_for_state(name)
        ):
            self.send_real_goal(name, target)
            if self.task_stopped:
                return

        if evaluation.status == JointGoalMonitor.REACHED:
            max_error_deg = math.degrees(evaluation.max_error_rad or 0.0)
            self.get_logger().info(
                f'REAL_GOAL_REACHED state={name} '
                f'max_error_deg={max_error_deg:.2f} '
                f'stable_s={evaluation.stable_s:.2f}'
            )
            if name == 'RETURN_HOME':
                self.return_home_confirmed = True
                self.get_logger().info(
                    'RETURN_HOME_CONFIRMED '
                    f'measured_deg={self.format_degrees(self.current_joint_state)} '
                    f'max_error_deg={max_error_deg:.2f}'
                )

            self.real_goal_monitor = None
            self.real_goal_command_sent = False
            self.finish_state()
            return

        if now - self.real_goal_last_log_time >= self.real_goal_log_period_s:
            self.real_goal_last_log_time = now
            max_error = (
                'N/A'
                if evaluation.max_error_rad is None
                else f'{math.degrees(evaluation.max_error_rad):.2f}'
            )
            self.get_logger().info(
                f'REAL_GOAL_WAIT state={name} '
                f'current_deg={self.format_degrees(self.current_joint_state)} '
                f'max_error_deg={max_error} '
                f'stable_s={evaluation.stable_s:.2f}'
            )

    def start_state(self):
        if self.state_index >= len(self.sequence):
            if self.config.is_real_robot and not self.return_home_confirmed:
                self.fail_task(
                    'COMPLETION_BLOCKED: RETURN_HOME has not been confirmed '
                    'from fresh MEASURED feedback'
                )
                return

            self.task_finished = True
            self.task_running = False
            self.publish_task_state('COMPLETED')
            self.get_logger().info('========================================')
            self.get_logger().info('TASK 2 PICK-AND-PLACE COMPLETED')
            self.get_logger().info('========================================')
            return

        name, state_type, target, duration, hold_time = self.sequence[self.state_index]
        self.publish_task_state(name)
        self.get_logger().info(f'STATE: {name}')

        if state_type == 'motion':
            try:
                self.validate_pose_if_enabled(target)
                if self.config.is_real_robot:
                    self.start_real_goal(name, target)
                else:
                    self.start_motion(target, duration)
            except KinematicsError as error:
                self.fail_task(f'{name}: {error}')
                return

        elif state_type == 'gripper':
            self.publish_gripper_command(target)
            extra_settle = self.real_gripper_settle if self.config.is_real_robot else 0.0
            self.hold_until = self.now_seconds() + hold_time + extra_settle
        else:
            self.fail_task(f'UNKNOWN_STATE_TYPE: {state_type}')
            return

        self.state_started = True

    def finish_state(self):
        self.state_index += 1
        self.state_started = False

    def update(self):
        if (
            self.task_stopped
            or self.task_finished
            or not self.task_running
        ):
            return

        if not self.state_started:
            self.start_state()
            return

        name, state_type, target, _, hold_time = self.sequence[self.state_index]

        if state_type == 'motion':
            if self.config.is_real_robot:
                if self.real_goal_monitor is None:
                    return
                self.update_real_goal(name, target)
                return

            if self.motion_active:
                finished = self.update_motion()
                if finished:
                    self.hold_until = self.now_seconds() + hold_time
                    self.get_logger().info(f'Reached: {name}')
                return

            if self.now_seconds() < self.hold_until:
                return
            self.finish_state()
            return

        if state_type == 'gripper':
            if self.config.is_real_robot:
                if self.now_seconds() < self.hold_until:
                    return
                self.finish_state()
                return

            self.publish_gripper_command(target)
            if self.now_seconds() < self.hold_until:
                return
            self.finish_state()


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
