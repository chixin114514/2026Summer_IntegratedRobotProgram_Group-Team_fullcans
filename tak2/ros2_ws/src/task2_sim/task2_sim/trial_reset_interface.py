import math
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String

from task2_sim.real_goal_monitor import (
    JointGoalMonitor,
    radians_to_degrees,
    reset_ready,
    within_tolerance,
)
from task2_sim.runtime_config import Task2Config


class TrialResetInterface(Node):
    """Device-specific scene reset with closed-loop real HOME verification."""

    def __init__(self):
        super().__init__('task2_trial_reset_interface')

        config_dir = get_package_share_directory('task2_sim') + '/config'
        self.config = Task2Config(config_dir)
        common = self.config.communication['common']
        task = self.config.task
        real_device = self.config.device.get('real', {})

        self.real_home = [
            math.radians(float(value))
            for value in task['home']['joints_deg']
        ]
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

        simulation_reset = task['simulation_reset']
        self.object_name = simulation_reset['object_name']
        self.world_name = simulation_reset['world_name']
        point = simulation_reset['point_a_world']
        self.reset_x = float(point['x'])
        self.reset_y = float(point['y'])
        self.reset_z = float(point['z'])

        self.waiting_for_operator = False
        self.operator_ready = False
        self.robot_home_confirmed = False
        self.real_home_command_sent = False
        self.real_home_monitor = None
        self.real_home_last_log_time = 0.0

        self.current_joint_state = None
        self.last_measured_state_time = None
        self.robot_state_source = 'UNKNOWN'

        self.pending_sim_done = False
        self.sim_done_time = 0.0

        self.reset_done_pub = self.create_publisher(
            Bool,
            common['trial_reset_done_topic'],
            10,
        )
        self.real_home_pub = self.create_publisher(
            Float64MultiArray,
            common['requested_arm_command_topic'],
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

        self.reset_request_sub = self.create_subscription(
            Bool,
            common['trial_reset_request_topic'],
            self.reset_request_callback,
            10,
        )
        self.operator_reset_sub = self.create_subscription(
            Bool,
            common['operator_reset_done_topic'],
            self.operator_reset_callback,
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

        self.timer = self.create_timer(0.10, self.update)
        self.get_logger().info('Trial reset interface started.')
        self.get_logger().info(f'Mode: {self.config.mode_name()}')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def measured_feedback_fresh(self, now=None):
        if self.current_joint_state is None or self.last_measured_state_time is None:
            return False
        if now is None:
            now = self.now_seconds()
        return now - self.last_measured_state_time <= self.real_feedback_max_age_s

    @staticmethod
    def format_degrees(values):
        if values is None:
            return 'N/A'
        return '[' + ', '.join(
            f'{value:.2f}' for value in radians_to_degrees(values)
        ) + ']'

    def robot_state_source_callback(self, message):
        self.robot_state_source = message.data.strip()

    def robot_state_callback(self, message):
        if len(message.position) < 6:
            return
        if self.config.is_real_robot:
            source = message.header.frame_id.strip() or self.robot_state_source
            if source != 'MEASURED':
                return
        self.current_joint_state = [float(value) for value in message.position[:6]]
        if self.config.is_real_robot:
            stamp = message.header.stamp
            measured_time = float(stamp.sec) + float(stamp.nanosec) / 1e9
            self.last_measured_state_time = (
                measured_time if measured_time > 0.0 else self.now_seconds()
            )

    def publish_reset_done(self):
        message = Bool()
        message.data = True
        self.reset_done_pub.publish(message)
        self.get_logger().info('Trial reset completed.')

    def publish_fault(self, reason):
        self.waiting_for_operator = False
        self.operator_ready = False
        self.robot_home_confirmed = False
        self.real_home_monitor = None
        self.real_home_command_sent = False

        message = String()
        message.data = str(reason)
        self.task_fault_pub.publish(message)
        self.get_logger().error(str(reason))

    def reset_request_callback(self, message):
        if not message.data:
            return

        if self.config.is_simulation:
            self.reset_simulation_object()
            return

        if self.waiting_for_operator:
            self.get_logger().warn('Duplicate real reset request ignored.')
            return

        now = self.now_seconds()
        self.waiting_for_operator = True
        self.operator_ready = False
        self.robot_home_confirmed = False
        self.real_home_command_sent = False
        self.real_home_last_log_time = 0.0
        self.real_home_monitor = JointGoalMonitor(
            state_name='RESET_HOME',
            target=self.real_home,
            tolerance_rad=self.real_home_tolerance,
            stable_required_s=self.real_goal_stable_s,
            timeout_s=self.real_motion_timeout_s,
            feedback_max_age_s=self.real_feedback_max_age_s,
            started_at=now,
        )

        state = String()
        state.data = 'RESET_HOME'
        self.task_state_pub.publish(state)

        self.get_logger().warn('========================================')
        self.get_logger().warn('REAL ROBOT RESET STARTED')
        self.get_logger().warn(
            'HOME will be verified from fresh MEASURED JointState. '
            'An extra HOME command is sent only when the robot is not '
            'already within HOME tolerance.'
        )
        self.get_logger().warn('Place the physical object back at point A.')
        self.get_logger().warn(
            'Then run: ros2 topic pub --once /task2/operator_reset_done '
            'std_msgs/msg/Bool "{data: true}"'
        )
        self.get_logger().warn('========================================')

        if (
            self.measured_feedback_fresh(now)
            and within_tolerance(
                self.current_joint_state,
                self.real_home,
                self.real_home_tolerance,
            )
        ):
            self.get_logger().info(
                'RESET_HOME command skipped: robot already within HOME '
                'tolerance; verifying stability only.'
            )
        elif self.measured_feedback_fresh(now):
            self.send_home_once()
        else:
            self.get_logger().warn(
                'RESET_HOME waiting for fresh MEASURED feedback before '
                'sending any motion command.'
            )

    def send_home_once(self):
        if self.real_home_command_sent:
            return
        command = Float64MultiArray()
        command.data = list(self.real_home)
        self.real_home_pub.publish(command)
        self.real_home_command_sent = True
        self.get_logger().info(
            'REAL_GOAL_SENT state=RESET_HOME '
            f'target_deg={self.format_degrees(self.real_home)}'
        )

    def reset_simulation_object(self):
        service = f'/world/{self.world_name}/set_pose'
        request = (
            f'name: "{self.object_name}", '
            f'position: {{x: {self.reset_x}, y: {self.reset_y}, z: {self.reset_z}}}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        )
        self.get_logger().info('Resetting simulation object to A.')

        try:
            result = subprocess.run(
                [
                    'ign',
                    'service',
                    '-s',
                    service,
                    '--reqtype',
                    'ignition.msgs.Pose',
                    '--reptype',
                    'ignition.msgs.Boolean',
                    '--timeout',
                    '3000',
                    '--req',
                    request,
                ],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except Exception as error:
            self.publish_fault('SIMULATION_RESET_EXCEPTION: ' + str(error))
            return

        output = (result.stdout or '') + (result.stderr or '')
        if result.returncode != 0:
            self.publish_fault('SIMULATION_RESET_FAILED: ' + output.strip())
            return
        if 'false' in output.lower():
            self.publish_fault('SIMULATION_RESET_REJECTED')
            return

        self.pending_sim_done = True
        self.sim_done_time = self.now_seconds() + 0.40

    def operator_reset_callback(self, message):
        if not message.data or not self.config.is_real_robot:
            return
        if not self.waiting_for_operator:
            self.get_logger().warn(
                'Operator reset ignored: no reset is currently requested.'
            )
            return

        self.operator_ready = True
        self.get_logger().info(
            'REAL ROBOT RESET: operator confirmed object at point A.'
        )
        self.try_finish_real_reset()

    def try_finish_real_reset(self):
        if not self.config.is_real_robot or not self.waiting_for_operator:
            return
        if not reset_ready(self.robot_home_confirmed, self.operator_ready):
            return

        self.waiting_for_operator = False
        self.operator_ready = False
        self.real_home_monitor = None
        self.get_logger().info(
            'REAL ROBOT RESET COMPLETE: measured HOME confirmed and object '
            'confirmed at A.'
        )
        self.publish_reset_done()

    def update_real_reset(self, now):
        if not self.waiting_for_operator or self.real_home_monitor is None:
            return

        evaluation = self.real_home_monitor.evaluate(
            now,
            self.current_joint_state,
            self.last_measured_state_time,
        )

        if evaluation.status == JointGoalMonitor.FEEDBACK_STALE:
            age = (
                'N/A'
                if evaluation.feedback_age_s is None
                else f'{evaluation.feedback_age_s:.3f}'
            )
            self.publish_fault(
                'RESET_HOME_FEEDBACK_STALE '
                f'current_deg={self.format_degrees(self.current_joint_state)} '
                f'feedback_age_s={age}'
            )
            return

        if evaluation.status == JointGoalMonitor.TIMEOUT:
            max_error = (
                'N/A'
                if evaluation.max_error_rad is None
                else f'{math.degrees(evaluation.max_error_rad):.2f}'
            )
            self.publish_fault(
                'RESET_HOME_TIMEOUT '
                f'target_deg={self.format_degrees(self.real_home)} '
                f'current_deg={self.format_degrees(self.current_joint_state)} '
                f'max_error_deg={max_error}'
            )
            return

        if (
            not self.real_home_command_sent
            and evaluation.max_error_rad is not None
            and evaluation.max_error_rad > self.real_home_tolerance
        ):
            self.send_home_once()

        if evaluation.status == JointGoalMonitor.REACHED:
            self.robot_home_confirmed = True
            max_error_deg = math.degrees(evaluation.max_error_rad or 0.0)
            self.get_logger().info(
                'RESET_HOME_CONFIRMED '
                f'measured_deg={self.format_degrees(self.current_joint_state)} '
                f'max_error_deg={max_error_deg:.2f} '
                f'stable_s={evaluation.stable_s:.2f}'
            )
            self.real_home_monitor = None
            self.try_finish_real_reset()
            return

        if now - self.real_home_last_log_time >= self.real_goal_log_period_s:
            self.real_home_last_log_time = now
            max_error = (
                'N/A'
                if evaluation.max_error_rad is None
                else f'{math.degrees(evaluation.max_error_rad):.2f}'
            )
            self.get_logger().info(
                'REAL_GOAL_WAIT state=RESET_HOME '
                f'current_deg={self.format_degrees(self.current_joint_state)} '
                f'max_error_deg={max_error} '
                f'stable_s={evaluation.stable_s:.2f}'
            )

    def update(self):
        now = self.now_seconds()

        if self.config.is_real_robot:
            self.update_real_reset(now)
            return

        if not self.pending_sim_done:
            return
        if now < self.sim_done_time:
            return

        self.pending_sim_done = False
        self.publish_reset_done()


def main(args=None):
    rclpy.init(args=args)
    node = TrialResetInterface()

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
