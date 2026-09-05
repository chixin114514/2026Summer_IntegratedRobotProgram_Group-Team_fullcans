import json

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    String,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class ExperimentManager(Node):

    def __init__(self):

        super().__init__(
            'task2_experiment_manager'
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

        experiment = (
            self.config.task[
                'experiment'
            ]
        )

        self.total_trials = int(
            experiment[
                'total_trials'
            ]
        )

        self.required_successes = int(
            experiment[
                'required_successes'
            ]
        )

        self.current_trial = 0
        self.success_count = 0
        self.failure_count = 0

        self.waiting_for_result = False
        self.waiting_for_reset = False
        self.finished = False

        # -----------------------------------------------------
        # Publishers
        # -----------------------------------------------------

        self.task_start_pub = (
            self.create_publisher(
                Bool,
                common[
                    'task_start_topic'
                ],
                10,
            )
        )

        self.reset_request_pub = (
            self.create_publisher(
                Bool,
                common[
                    'trial_reset_request_topic'
                ],
                10,
            )
        )

        self.experiment_state_pub = (
            self.create_publisher(
                String,
                '/task2/experiment_state',
                10,
            )
        )

        # -----------------------------------------------------
        # Subscribers
        # -----------------------------------------------------

        self.task_state_sub = (
            self.create_subscription(
                String,
                common[
                    'task_state_topic'
                ],
                self.task_state_callback,
                10,
            )
        )

        self.trial_result_sub = (
            self.create_subscription(
                String,
                common[
                    'trial_result_topic'
                ],
                self.trial_result_callback,
                10,
            )
        )

        self.reset_done_sub = (
            self.create_subscription(
                Bool,
                common[
                    'trial_reset_done_topic'
                ],
                self.reset_done_callback,
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

        # Do NOT start on a fixed timer.
        #
        # The experiment may start only after task_manager has
        # finished IK / limit validation and publishes READY.
        self.manager_ready = False

        self.get_logger().info(
            'Experiment manager started.'
        )

        self.get_logger().info(
            f'Acceptance target: '
            f'{self.required_successes}/'
            f'{self.total_trials}'
        )

    # ========================================================
    # Helpers
    # ========================================================

    def publish_state(
        self,
        state,
        extra=None,
    ):

        data = {
            'state': state,
            'trial': self.current_trial,
            'successes': self.success_count,
            'failures': self.failure_count,
            'total_trials': self.total_trials,
            'required_successes':
                self.required_successes,
            'mode':
                self.config.mode_name(),
        }

        if extra is not None:

            data.update(
                extra
            )

        message = String()

        message.data = json.dumps(
            data
        )

        self.experiment_state_pub.publish(
            message
        )

    # ========================================================
    # Trial control
    # ========================================================

    def start_next_trial(self):

        if self.finished:
            return

        if (
            self.current_trial
            >=
            self.total_trials
        ):

            self.finish_experiment()

            return

        self.current_trial += 1

        self.waiting_for_result = False
        self.waiting_for_reset = False

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            f'START TRIAL '
            f'{self.current_trial}/'
            f'{self.total_trials}'
        )

        self.get_logger().info(
            '================================'
        )

        self.publish_state(
            'TRIAL_STARTING'
        )

        message = Bool()
        message.data = True

        self.task_start_pub.publish(
            message
        )

    # ========================================================
    # Task completion
    # ========================================================

    def task_state_callback(
        self,
        message,
    ):

        state = (
            message.data.strip()
        )

        if state == 'READY':

            # First READY starts the acceptance experiment.
            #
            # Repeated READY announcements are intentionally
            # ignored after the experiment has started.

            if (
                not self.manager_ready
                and
                not self.finished
            ):

                self.manager_ready = True

                self.get_logger().info(
                    'Task manager READY.'
                )

                # -------------------------------------------------
                # Trial 1 also requires a clean scene.
                #
                # Do exactly the same reset used between later
                # trials:
                #
                # simulation:
                #   target_object -> A
                #
                # real robot:
                #   operator prepares the object at A
                #
                # Only after reset_done may Trial 1 start.
                # -------------------------------------------------

                self.waiting_for_reset = True

                reset = Bool()
                reset.data = True

                self.reset_request_pub.publish(
                    reset
                )

                self.publish_state(
                    'INITIAL_SCENE_RESET'
                )

                self.get_logger().info(
                    'Preparing initial object at point A.'
                )

            return

        if state == 'COMPLETED':

            if self.waiting_for_result:
                return

            self.waiting_for_result = True

            self.get_logger().info(
                f'Trial {self.current_trial}: '
                'motion completed; waiting for '
                'success evaluation.'
            )

            self.publish_state(
                'WAITING_FOR_RESULT'
            )

        elif (
            state.startswith(
                'ERROR:'
            )
        ):

            # If Trial 1 never started, this is a system
            # initialisation error rather than a grasp failure.

            if self.current_trial == 0:

                self.finished = True

                self.get_logger().error(
                    'Experiment cannot start: '
                    + state
                )

                self.publish_state(
                    'INITIALISATION_FAILED',
                    {
                        'reason':
                            state,
                    },
                )

                return

            if self.waiting_for_result:
                return

            self.waiting_for_result = True

            # A task error is automatically a failed trial.

            result = String()

            result.data = json.dumps({
                'trial':
                    self.current_trial,

                'success':
                    False,

                'reason':
                    state,
            })

            self.trial_result_callback(
                result
            )

    # ========================================================
    # Result
    # ========================================================

    def trial_result_callback(
        self,
        message,
    ):

        if self.finished:
            return

        if not self.waiting_for_result:
            return

        try:

            data = json.loads(
                message.data
            )

            success = bool(
                data.get(
                    'success',
                    False,
                )
            )

            reason = str(
                data.get(
                    'reason',
                    '',
                )
            )

        except Exception:

            success = False

            reason = (
                'INVALID_RESULT_MESSAGE'
            )

        if success:

            self.success_count += 1

            result_text = 'SUCCESS'

        else:

            self.failure_count += 1

            result_text = 'FAILED'

        self.get_logger().info(
            f'Trial {self.current_trial}: '
            f'{result_text}'
        )

        self.publish_state(
            'TRIAL_RESULT',
            {
                'trial_success':
                    success,

                'reason':
                    reason,
            },
        )

        self.waiting_for_result = False

        if (
            self.current_trial
            >=
            self.total_trials
        ):

            self.finish_experiment()

            return

        # Request the device-specific backend to prepare A
        # for the next trial.
        #
        # mode 0:
        #   reset simulation object to point A
        #
        # mode 1:
        #   wait for operator to place object at A

        self.waiting_for_reset = True

        reset = Bool()
        reset.data = True

        self.reset_request_pub.publish(
            reset
        )

        self.publish_state(
            'WAITING_FOR_RESET'
        )

    # ========================================================
    # Reset complete
    # ========================================================

    def reset_done_callback(
        self,
        message,
    ):

        if not message.data:
            return

        if not self.waiting_for_reset:
            return

        self.waiting_for_reset = False

        self.start_next_trial()

    # ========================================================
    # Safety
    # ========================================================

    def safety_stop_callback(
        self,
        message,
    ):

        if not message.data:
            return

        if self.finished:
            return

        self.get_logger().error(
            'Experiment aborted by SAFE_STOP.'
        )

        self.finished = True

        self.publish_state(
            'ABORTED_SAFE_STOP'
        )

    # ========================================================
    # Final result
    # ========================================================

    def finish_experiment(self):

        self.finished = True

        passed = (
            self.success_count
            >=
            self.required_successes
        )

        result = (
            'PASS'
            if passed
            else
            'FAIL'
        )

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'TASK 2 ACCEPTANCE RESULT'
        )

        self.get_logger().info(
            f'Success: '
            f'{self.success_count}/'
            f'{self.total_trials}'
        )

        self.get_logger().info(
            f'Required: '
            f'{self.required_successes}/'
            f'{self.total_trials}'
        )

        self.get_logger().info(
            f'Result: {result}'
        )

        self.get_logger().info(
            '================================'
        )

        self.publish_state(
            'COMPLETED',
            {
                'passed':
                    passed,

                'result':
                    result,
            },
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ExperimentManager()

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
