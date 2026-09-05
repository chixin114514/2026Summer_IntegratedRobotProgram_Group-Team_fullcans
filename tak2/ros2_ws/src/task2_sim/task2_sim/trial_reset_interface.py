import subprocess

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

            self.waiting_for_operator = True

            self.get_logger().warn(
                'REAL ROBOT: place the physical object '
                'back at point A.'
            )

            self.get_logger().warn(
                'After the object is ready, run:'
            )

            self.get_logger().warn(
                'ros2 topic pub --once '
                '/task2/operator_reset_done '
                'std_msgs/msg/Bool "{data: true}"'
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

        self.sim_done_time = (
            self.now_seconds()
            +
            1.0
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

        self.waiting_for_operator = False

        self.publish_reset_done()

    # ========================================================
    # Timer
    # ========================================================

    def update(self):

        if not self.pending_sim_done:

            return

        if (
            self.now_seconds()
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

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
