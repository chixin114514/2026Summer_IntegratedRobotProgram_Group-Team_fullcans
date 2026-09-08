import math
import subprocess

import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    Float64MultiArray,
    String,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class TrialResetInterface(Node):

    def __init__(self):

        super().__init__(
            'task2_trial_reset_interface'
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

        task = self.config.task

        # =====================================================
        # Real-robot reset configuration
        #
        # Physical reset consists of:
        #
        #   1. command robot back to HOME
        #   2. operator restores the physical object to A
        #   3. publish trial_reset_done only after BOTH finish
        # =====================================================

        self.real_home = [
            math.radians(
                float(value)
            )
            for value in task[
                'home'
            ][
                'joints_deg'
            ]
        ]

        motion = task[
            'motion'
        ]

        real_device = (
            self.config.device.get(
                'real',
                {},
            )
        )

        calculated_home_wait = (
            float(
                motion.get(
                    'home_duration_s',
                    1.5,
                )
            )
            *
            float(
                motion.get(
                    'real_motion_duration_scale',
                    1.0,
                )
            )
            +
            float(
                real_device.get(
                    'motion_settle_s',
                    0.5,
                )
            )
        )

        # The real arm currently runs at low speed.
        # Give HOME enough time during acceptance reset.
        self.real_home_wait_s = max(
            5.0,
            calculated_home_wait,
        )

        simulation_reset = (
            task[
                'simulation_reset'
            ]
        )

        self.object_name = (
            simulation_reset[
                'object_name'
            ]
        )

        self.world_name = (
            simulation_reset[
                'world_name'
            ]
        )

        point = (
            simulation_reset[
                'point_a_world'
            ]
        )

        self.reset_x = float(
            point[
                'x'
            ]
        )

        self.reset_y = float(
            point[
                'y'
            ]
        )

        self.reset_z = float(
            point[
                'z'
            ]
        )

        self.waiting_for_operator = False

        self.operator_ready = False

        self.real_home_command_pending = False

        self.real_home_command_time = 0.0

        self.real_home_pending = False

        self.real_home_done_time = 0.0

        self.pending_sim_done = False

        self.sim_done_time = 0.0

        # -----------------------------------------------------
        # Output
        # -----------------------------------------------------

        self.reset_done_pub = (
            self.create_publisher(
                Bool,
                common[
                    'trial_reset_done_topic'
                ],
                10,
            )
        )

        # Real HOME reset goes through the normal Task2
        # arm command -> safety monitor -> arm interface path.
        self.real_home_pub = (
            self.create_publisher(
                Float64MultiArray,
                common[
                    'requested_arm_command_topic'
                ],
                10,
            )
        )

        # RESET_HOME is also visible in the normal task-state
        # stream for logging and safety synchronisation.
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

        # -----------------------------------------------------
        # Input
        # -----------------------------------------------------

        self.reset_request_sub = (
            self.create_subscription(
                Bool,
                common[
                    'trial_reset_request_topic'
                ],
                self.reset_request_callback,
                10,
            )
        )

        self.operator_reset_sub = (
            self.create_subscription(
                Bool,
                common[
                    'operator_reset_done_topic'
                ],
                self.operator_reset_callback,
                10,
            )
        )

        self.timer = self.create_timer(
            0.10,
            self.update,
        )

        self.get_logger().info(
            'Trial reset interface started.'
        )

        self.get_logger().info(
            f'Mode: '
            f'{self.config.mode_name()}'
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
    # Complete reset
    # ========================================================

    def publish_reset_done(self):

        message = Bool()

        message.data = True

        self.reset_done_pub.publish(
            message
        )

        self.get_logger().info(
            'Trial reset completed.'
        )

    # ========================================================
    # Fault
    # ========================================================

    def publish_fault(
        self,
        reason,
    ):

        message = String()

        message.data = str(
            reason
        )

        self.task_fault_pub.publish(
            message
        )

        self.get_logger().error(
            str(reason)
        )

    # ========================================================
    # Reset request
    # ========================================================

    def reset_request_callback(
        self,
        message,
    ):

        if not message.data:

            return

        if self.config.is_simulation:

            self.reset_simulation_object()

        else:

            # =================================================
            # REAL ROBOT RESET
            #
            # Do not immediately declare reset complete.
            #
            # Condition A:
            #     robot HOME reset complete
            #
            # Condition B:
            #     operator confirms object restored to A
            #
            # Only A + B -> trial_reset_done
            # =================================================

            self.waiting_for_operator = True

            self.operator_ready = False

            self.real_home_pending = False

            # Publish RESET_HOME first. The actual HOME command
            # is sent 0.20 s later so safety_monitor has time to
            # reset its command-step reference.
            self.real_home_command_pending = True

            self.real_home_command_time = (
                self.now_seconds()
                +
                0.20
            )

            state = String()

            state.data = 'RESET_HOME'

            self.task_state_pub.publish(
                state
            )

            self.get_logger().warn(
                '========================================'
            )

            self.get_logger().warn(
                'REAL ROBOT RESET STARTED'
            )

            self.get_logger().warn(
                'Robot will automatically return to HOME.'
            )

            self.get_logger().warn(
                'Place the physical object back at point A.'
            )

            self.get_logger().warn(
                'After the object is ready, confirm with:'
            )

            self.get_logger().warn(
                'ros2 topic pub --once '
                '/task2/operator_reset_done '
                'std_msgs/msg/Bool "{data: true}"'
            )

            self.get_logger().warn(
                'Next trial starts only after HOME + '
                'object reset are both complete.'
            )

            self.get_logger().warn(
                '========================================'
            )

    # ========================================================
    # Gazebo reset
    # ========================================================

    def reset_simulation_object(self):

        service = (
            f'/world/'
            f'{self.world_name}'
            f'/set_pose'
        )

        request = (
            f'name: "{self.object_name}", '
            f'position: {{'
            f'x: {self.reset_x}, '
            f'y: {self.reset_y}, '
            f'z: {self.reset_z}'
            f'}}, '
            f'orientation: {{'
            f'x: 0.0, '
            f'y: 0.0, '
            f'z: 0.0, '
            f'w: 1.0'
            f'}}'
        )

        self.get_logger().info(
            'Resetting simulation object to A.'
        )

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

            self.publish_fault(
                'SIMULATION_RESET_EXCEPTION: '
                +
                str(error)
            )

            return

        output = (
            (
                result.stdout
                or
                ''
            )
            +
            (
                result.stderr
                or
                ''
            )
        )

        if (
            result.returncode
            !=
            0
        ):

            self.publish_fault(
                'SIMULATION_RESET_FAILED: '
                +
                output.strip()
            )

            return

        if (
            'false'
            in
            output.lower()
        ):

            self.publish_fault(
                'SIMULATION_RESET_REJECTED'
            )

            return

        # Give Gazebo time to apply pose and let the block
        # settle on the table before the next trial starts.

        self.pending_sim_done = True

        # The object needs only a short settling period after
        # being teleported back to point A.
        self.sim_done_time = (
            self.now_seconds()
            +
            0.40
        )

    # ========================================================
    # Real robot operator reset
    # ========================================================

    def operator_reset_callback(
        self,
        message,
    ):

        if not message.data:

            return

        if not self.config.is_real_robot:

            return

        if not self.waiting_for_operator:

            self.get_logger().warn(
                'Operator reset ignored: '
                'no reset is currently requested.'
            )

            return

        self.operator_ready = True

        self.get_logger().info(
            'REAL ROBOT RESET: '
            'operator confirmed object at point A.'
        )

        self.try_finish_real_reset()

    # ========================================================
    # Real reset completion
    # ========================================================

    def try_finish_real_reset(
        self,
    ):

        if not self.config.is_real_robot:

            return

        if not self.waiting_for_operator:

            return

        if not self.operator_ready:

            return

        if self.real_home_command_pending:

            return

        if self.real_home_pending:

            return

        self.waiting_for_operator = False

        self.operator_ready = False

        self.get_logger().info(
            'REAL ROBOT RESET COMPLETE: '
            'HOME reached and object confirmed at A.'
        )

        self.publish_reset_done()

    # ========================================================
    # Timer
    # ========================================================

    def update(self):

        now = self.now_seconds()

        # =====================================================
        # REAL ROBOT
        # =====================================================

        if self.config.is_real_robot:

            # Send HOME once after RESET_HOME state has had
            # time to propagate through ROS2.
            if (
                self.real_home_command_pending
                and
                now
                >=
                self.real_home_command_time
            ):

                command = Float64MultiArray()

                command.data = list(
                    self.real_home
                )

                self.real_home_pub.publish(
                    command
                )

                self.real_home_command_pending = False

                self.real_home_pending = True

                self.real_home_done_time = (
                    now
                    +
                    self.real_home_wait_s
                )

                self.get_logger().info(
                    'REAL ROBOT RESET: '
                    'HOME command sent.'
                )

                self.get_logger().info(
                    'REAL ROBOT RESET: '
                    f'waiting {self.real_home_wait_s:.2f}s '
                    'for HOME motion.'
                )

            if (
                self.real_home_pending
                and
                now
                >=
                self.real_home_done_time
            ):

                self.real_home_pending = False

                self.get_logger().info(
                    'REAL ROBOT RESET: '
                    'HOME motion window complete.'
                )

                self.try_finish_real_reset()

            return

        # =====================================================
        # SIMULATION
        # =====================================================

        if not self.pending_sim_done:

            return

        if (
            now
            <
            self.sim_done_time
        ):

            return

        self.pending_sim_done = False

        self.publish_reset_done()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = TrialResetInterface()

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
