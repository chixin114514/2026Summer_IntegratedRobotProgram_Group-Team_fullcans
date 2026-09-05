import csv
import json

from datetime import datetime
from pathlib import Path

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import (
    Bool,
    String,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class TaskLogger(Node):

    def __init__(self):

        super().__init__(
            'task2_logger'
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

        # -----------------------------------------------------
        # Result directory
        # -----------------------------------------------------

        timestamp = (
            datetime.now()
            .strftime(
                '%Y%m%d_%H%M%S'
            )
        )

        self.result_dir = (
            Path.home()
            /
            'task2_results'
        )

        self.result_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.trajectory_path = (
            self.result_dir
            /
            f'trajectory_{timestamp}.csv'
        )

        self.result_path = (
            self.result_dir
            /
            f'task_results_{timestamp}.csv'
        )

        self.error_path = (
            self.result_dir
            /
            f'errors_{timestamp}.log'
        )

        # -----------------------------------------------------
        # CSV initialisation
        # -----------------------------------------------------

        with self.trajectory_path.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as file:

            writer = csv.writer(
                file
            )

            writer.writerow([
                'time_s',
                'task_state',
                'state_source',
                'j1_rad',
                'j2_rad',
                'j3_rad',
                'j4_rad',
                'j5_rad',
                'j6_rad',
            ])

        with self.result_path.open(
            'w',
            newline='',
            encoding='utf-8',
        ) as file:

            writer = csv.writer(
                file
            )

            writer.writerow([
                'time',
                'trial',
                'success',
                'reason',
                'successes',
                'failures',
            ])

        self.error_path.touch()

        # -----------------------------------------------------
        # Runtime
        # -----------------------------------------------------

        self.start_time = (
            self.now_seconds()
        )

        self.task_state = 'UNKNOWN'

        self.state_source = 'UNKNOWN'

        # -----------------------------------------------------
        # Subscribers
        # -----------------------------------------------------

        self.robot_state_sub = (
            self.create_subscription(
                JointState,
                common[
                    'robot_state_topic'
                ],
                self.robot_state_callback,
                50,
            )
        )

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

        self.state_source_sub = (
            self.create_subscription(
                String,
                common[
                    'robot_state_source_topic'
                ],
                self.state_source_callback,
                10,
            )
        )

        self.safety_state_sub = (
            self.create_subscription(
                String,
                common[
                    'safety_state_topic'
                ],
                self.safety_state_callback,
                10,
            )
        )

        self.task_fault_sub = (
            self.create_subscription(
                String,
                common[
                    'task_fault_topic'
                ],
                self.task_fault_callback,
                10,
            )
        )

        self.experiment_state_sub = (
            self.create_subscription(
                String,
                '/task2/experiment_state',
                self.experiment_state_callback,
                10,
            )
        )

        self.get_logger().info(
            'Task logger started.'
        )

        self.get_logger().info(
            f'Results directory: '
            f'{self.result_dir}'
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

    def timestamp(self):

        return (
            datetime.now()
            .isoformat(
                timespec='milliseconds'
            )
        )

    # ========================================================
    # Trajectory
    # ========================================================

    def robot_state_callback(
        self,
        message,
    ):

        if len(
            message.position
        ) < 6:

            return

        elapsed = (
            self.now_seconds()
            -
            self.start_time
        )

        row = [

            f'{elapsed:.4f}',

            self.task_state,

            self.state_source,

        ]

        row.extend([
            f'{float(value):.8f}'
            for value in (
                message.position[
                    :6
                ]
            )
        ])

        with self.trajectory_path.open(
            'a',
            newline='',
            encoding='utf-8',
        ) as file:

            csv.writer(
                file
            ).writerow(
                row
            )

    # ========================================================
    # State
    # ========================================================

    def task_state_callback(
        self,
        message,
    ):

        self.task_state = (
            message.data
        )

    def state_source_callback(
        self,
        message,
    ):

        self.state_source = (
            message.data
        )

    # ========================================================
    # Errors
    # ========================================================

    def append_error(
        self,
        source,
        message,
    ):

        line = (
            f'[{self.timestamp()}] '
            f'[{source}] '
            f'{message}\n'
        )

        with self.error_path.open(
            'a',
            encoding='utf-8',
        ) as file:

            file.write(
                line
            )

    def safety_state_callback(
        self,
        message,
    ):

        if message.data.startswith(
            'SAFE_STOP'
        ):

            self.append_error(
                'SAFETY',
                message.data,
            )

    def task_fault_callback(
        self,
        message,
    ):

        self.append_error(
            'TASK',
            message.data,
        )

    # ========================================================
    # Trial / acceptance result
    # ========================================================

    def experiment_state_callback(
        self,
        message,
    ):

        try:

            data = json.loads(
                message.data
            )

        except Exception:

            self.append_error(
                'LOGGER',
                'Invalid experiment state JSON.',
            )

            return

        if (
            data.get(
                'state'
            )
            !=
            'TRIAL_RESULT'
        ):

            return

        with self.result_path.open(
            'a',
            newline='',
            encoding='utf-8',
        ) as file:

            writer = csv.writer(
                file
            )

            writer.writerow([

                self.timestamp(),

                data.get(
                    'trial',
                    '',
                ),

                data.get(
                    'trial_success',
                    False,
                ),

                data.get(
                    'reason',
                    '',
                ),

                data.get(
                    'successes',
                    0,
                ),

                data.get(
                    'failures',
                    0,
                ),
            ])


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TaskLogger()

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
