import json

import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String

from task2_sim.runtime_config import Task2Config


class ExperimentManager(Node):
    def __init__(self):
        super().__init__('task2_experiment_manager')

        config_dir = get_package_share_directory('task2_sim') + '/config'
        self.config = Task2Config(config_dir)
        common = self.config.communication['common']
        experiment = self.config.task['experiment']

        self.total_trials = int(experiment['total_trials'])
        self.required_successes = int(experiment['required_successes'])
        self.current_trial = 0
        self.success_count = 0
        self.failure_count = 0
        self.waiting_for_result = False
        self.waiting_for_reset = False
        self.finished = False
        self.manager_ready = False

        self.task_start_pub = self.create_publisher(
            Bool,
            common['task_start_topic'],
            10,
        )
        self.reset_request_pub = self.create_publisher(
            Bool,
            common['trial_reset_request_topic'],
            10,
        )
        self.experiment_state_pub = self.create_publisher(
            String,
            '/task2/experiment_state',
            10,
        )

        self.task_state_sub = self.create_subscription(
            String,
            common['task_state_topic'],
            self.task_state_callback,
            10,
        )
        self.trial_result_sub = self.create_subscription(
            String,
            common['trial_result_topic'],
            self.trial_result_callback,
            10,
        )
        self.reset_done_sub = self.create_subscription(
            Bool,
            common['trial_reset_done_topic'],
            self.reset_done_callback,
            10,
        )
        self.safety_stop_sub = self.create_subscription(
            Bool,
            common['safety_stop_topic'],
            self.safety_stop_callback,
            10,
        )
        self.task_fault_sub = self.create_subscription(
            String,
            common['task_fault_topic'],
            self.task_fault_callback,
            10,
        )

        self.get_logger().info('Experiment manager started.')
        self.get_logger().info(
            f'Acceptance target: {self.required_successes}/{self.total_trials}'
        )

    def publish_state(self, state, extra=None):
        data = {
            'state': state,
            'trial': self.current_trial,
            'successes': self.success_count,
            'failures': self.failure_count,
            'total_trials': self.total_trials,
            'required_successes': self.required_successes,
            'mode': self.config.mode_name(),
        }
        if extra is not None:
            data.update(extra)
        message = String()
        message.data = json.dumps(data)
        self.experiment_state_pub.publish(message)

    def start_next_trial(self):
        if self.finished:
            return
        if self.current_trial >= self.total_trials:
            self.finish_experiment()
            return

        self.current_trial += 1
        self.waiting_for_result = False
        self.waiting_for_reset = False
        self.get_logger().info('================================')
        self.get_logger().info(
            f'START TRIAL {self.current_trial}/{self.total_trials}'
        )
        self.get_logger().info('================================')
        self.publish_state('TRIAL_STARTING')

        message = Bool()
        message.data = True
        self.task_start_pub.publish(message)

    def task_state_callback(self, message):
        state = message.data.strip()

        if state == 'READY':
            if not self.manager_ready and not self.finished:
                self.manager_ready = True
                self.waiting_for_reset = True
                reset = Bool()
                reset.data = True
                self.reset_request_pub.publish(reset)
                self.publish_state('INITIAL_SCENE_RESET')
                self.get_logger().info(
                    'Preparing initial object and verifying real HOME.'
                )
            return

        if self.finished:
            return

        if state == 'COMPLETED':
            if self.waiting_for_result:
                return
            self.waiting_for_result = True
            self.get_logger().info(
                f'Trial {self.current_trial}: motion completed with '
                'RETURN_HOME confirmed; waiting for success evaluation.'
            )
            self.publish_state('WAITING_FOR_RESULT')
            return

        if state.startswith('ERROR:'):
            self.abort('ABORTED_TASK_ERROR', state)

    def task_fault_callback(self, message):
        reason = message.data.strip() or 'UNSPECIFIED_TASK_FAULT'
        self.abort('ABORTED_TASK_FAULT', reason)

    def abort(self, state, reason):
        if self.finished:
            return
        self.finished = True
        self.waiting_for_result = False
        self.waiting_for_reset = False
        self.get_logger().error(
            f'Experiment aborted: {reason}. Relaunch is required before '
            'another acceptance run.'
        )
        self.publish_state(state, {'reason': reason})

    def trial_result_callback(self, message):
        if self.finished or not self.waiting_for_result:
            return

        try:
            data = json.loads(message.data)
            success = bool(data.get('success', False))
            reason = str(data.get('reason', ''))
        except Exception:
            success = False
            reason = 'INVALID_RESULT_MESSAGE'

        if success:
            self.success_count += 1
            result_text = 'SUCCESS'
        else:
            self.failure_count += 1
            result_text = 'FAILED'

        self.get_logger().info(
            f'Trial {self.current_trial}: {result_text}'
        )
        self.publish_state(
            'TRIAL_RESULT',
            {'trial_success': success, 'reason': reason},
        )
        self.waiting_for_result = False

        if self.current_trial >= self.total_trials:
            self.finish_experiment()
            return

        self.waiting_for_reset = True
        reset = Bool()
        reset.data = True
        self.reset_request_pub.publish(reset)
        self.publish_state('WAITING_FOR_RESET')

    def reset_done_callback(self, message):
        if not message.data or self.finished or not self.waiting_for_reset:
            return
        self.waiting_for_reset = False
        self.start_next_trial()

    def safety_stop_callback(self, message):
        if message.data:
            self.abort('ABORTED_SAFE_STOP', 'SAFE_STOP')

    def finish_experiment(self):
        if self.finished:
            return
        self.finished = True
        passed = self.success_count >= self.required_successes
        result = 'PASS' if passed else 'FAIL'

        self.get_logger().info('================================')
        self.get_logger().info('TASK 2 ACCEPTANCE RESULT')
        self.get_logger().info(
            f'Success: {self.success_count}/{self.total_trials}'
        )
        self.get_logger().info(
            f'Required: {self.required_successes}/{self.total_trials}'
        )
        self.get_logger().info(f'Result: {result}')
        self.get_logger().info('================================')
        self.publish_state(
            'COMPLETED',
            {'passed': passed, 'result': result},
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentManager()

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
