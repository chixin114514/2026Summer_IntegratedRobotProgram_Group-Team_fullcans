import json
import math

import rclpy

from ament_index_python.packages import (
    get_package_share_directory,
)

from rclpy.node import Node

from std_msgs.msg import (
    Bool,
    String,
)

from tf2_msgs.msg import (
    TFMessage,
)

from task2_sim.runtime_config import (
    Task2Config,
)


class ResultMonitor(Node):

    def __init__(self):

        super().__init__(
            'task2_result_monitor'
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

        acceptance = (
            task[
                'acceptance'
            ]
        )

        self.point_b_x = float(
            task[
                'point_b'
            ][
                'x'
            ]
        )

        self.point_b_y = float(
            task[
                'point_b'
            ][
                'y'
            ]
        )

        self.expected_z = float(
            acceptance[
                'simulation_object_center_z_m'
            ]
        )

        self.xy_tolerance = float(
            acceptance[
                'placement_xy_tolerance_m'
            ]
        )

        self.z_tolerance = float(
            acceptance[
                'placement_z_tolerance_m'
            ]
        )

        self.maximum_tilt = math.radians(
            float(
                acceptance[
                    'maximum_tilt_deg'
                ]
            )
        )

        self.settle_time = float(
            acceptance[
                'settle_time_s'
            ]
        )

        self.pose_timeout = float(
            acceptance[
                'object_pose_timeout_s'
            ]
        )

        self.object_name = (
            task[
                'simulation_reset'
            ][
                'object_name'
            ]
        )

        # -----------------------------------------------------
        # Runtime
        # -----------------------------------------------------

        self.pending_result = False

        self.evaluate_after = 0.0

        self.last_object_transform = None

        self.last_object_pose_time = None

        # -----------------------------------------------------
        # Output
        # -----------------------------------------------------

        self.result_pub = (
            self.create_publisher(
                String,
                common[
                    'trial_result_topic'
                ],
                10,
            )
        )

        # -----------------------------------------------------
        # Task completion
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

        # -----------------------------------------------------
        # Mode-specific result source
        # -----------------------------------------------------

        if self.config.is_simulation:

            pose_topic = (
                self.config.communication[
                    'simulation'
                ][
                    'object_pose_topic'
                ]
            )

            self.pose_sub = (
                self.create_subscription(
                    TFMessage,
                    pose_topic,
                    self.pose_callback,
                    10,
                )
            )

            self.operator_result_sub = None

            self.get_logger().info(
                'Result monitor mode: '
                'SIMULATION / automatic pose evaluation'
            )

        else:

            self.pose_sub = None

            self.operator_result_sub = (
                self.create_subscription(
                    Bool,
                    common[
                        'operator_trial_result_topic'
                    ],
                    self.operator_result_callback,
                    10,
                )
            )

            self.get_logger().info(
                'Result monitor mode: '
                'REAL ROBOT / operator confirmation'
            )

        self.timer = self.create_timer(
            0.10,
            self.update,
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
    # Task completion
    # ========================================================

    def task_state_callback(
        self,
        message,
    ):

        if (
            message.data.strip()
            !=
            'COMPLETED'
        ):

            return

        self.pending_result = True

        self.evaluate_after = (
            self.now_seconds()
            +
            self.settle_time
        )

        if self.config.is_simulation:

            self.get_logger().info(
                'Task completed. Waiting for '
                'object to settle before evaluation.'
            )

        else:

            self.get_logger().info(
                'Task completed on REAL ROBOT.'
            )

            self.get_logger().info(
                'Confirm result with:'
            )

            self.get_logger().info(
                'SUCCESS: ros2 topic pub --once '
                '/task2/operator_trial_result '
                'std_msgs/msg/Bool "{data: true}"'
            )

            self.get_logger().info(
                'FAILED: ros2 topic pub --once '
                '/task2/operator_trial_result '
                'std_msgs/msg/Bool "{data: false}"'
            )

    # ========================================================
    # Gazebo object pose
    # ========================================================

    def pose_callback(
        self,
        message,
    ):

        best = None

        for transform in (
            message.transforms
        ):

            child = (
                transform.child_frame_id
                or
                ''
            )

            # Pose_V can contain both model and link poses.
            #
            # Accept the target model or its object_link.
            if (
                self.object_name
                not in child
            ):

                continue

            # Prefer the model-level transform when possible.
            if (
                child
                ==
                self.object_name
            ):

                best = transform

                break

            if best is None:

                best = transform

        if best is None:

            return

        self.last_object_transform = best

        self.last_object_pose_time = (
            self.now_seconds()
        )

    # ========================================================
    # Quaternion tilt
    #
    # Compare the block local Z axis with the world Z axis.
    # ========================================================

    @staticmethod
    def calculate_tilt(
        quaternion,
    ):

        x = float(
            quaternion.x
        )

        y = float(
            quaternion.y
        )

        z = float(
            quaternion.z
        )

        w = float(
            quaternion.w
        )

        norm = math.sqrt(
            x * x
            +
            y * y
            +
            z * z
            +
            w * w
        )

        if norm < 1e-12:

            return math.pi

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        # Z component of local Z axis after quaternion rotation.
        local_z_world_z = (
            1.0
            -
            2.0
            *
            (
                x * x
                +
                y * y
            )
        )

        local_z_world_z = max(
            -1.0,
            min(
                1.0,
                local_z_world_z,
            ),
        )

        return math.acos(
            local_z_world_z
        )

    # ========================================================
    # Publish result
    # ========================================================

    def publish_result(
        self,
        success,
        reason,
        extra=None,
    ):

        data = {

            'success':
                bool(success),

            'reason':
                str(reason),

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

        self.result_pub.publish(
            message
        )

        result_text = (
            'SUCCESS'
            if success
            else
            'FAILED'
        )

        self.get_logger().info(
            f'Trial evaluation: '
            f'{result_text} - {reason}'
        )

        self.pending_result = False

    # ========================================================
    # Simulation evaluation
    # ========================================================

    def evaluate_simulation(self):

        if (
            self.last_object_transform
            is None
            or
            self.last_object_pose_time
            is None
        ):

            self.publish_result(
                False,
                'OBJECT_POSE_UNAVAILABLE',
            )

            return

        pose_age = (
            self.now_seconds()
            -
            self.last_object_pose_time
        )

        if (
            pose_age
            >
            self.pose_timeout
        ):

            self.publish_result(
                False,
                'OBJECT_POSE_STALE',
                {
                    'pose_age_s':
                        pose_age,
                },
            )

            return

        transform = (
            self.last_object_transform
            .transform
        )

        x = float(
            transform.translation.x
        )

        y = float(
            transform.translation.y
        )

        z = float(
            transform.translation.z
        )

        xy_error = math.hypot(
            x - self.point_b_x,
            y - self.point_b_y,
        )

        z_error = abs(
            z
            -
            self.expected_z
        )

        tilt = (
            self.calculate_tilt(
                transform.rotation
            )
        )

        success = (
            xy_error
            <=
            self.xy_tolerance

            and

            z_error
            <=
            self.z_tolerance

            and

            tilt
            <=
            self.maximum_tilt
        )

        if success:

            reason = (
                'OBJECT_INSIDE_B_AND_STABLE'
            )

        elif (
            xy_error
            >
            self.xy_tolerance
        ):

            reason = (
                'OBJECT_OUTSIDE_B'
            )

        elif (
            z_error
            >
            self.z_tolerance
        ):

            reason = (
                'OBJECT_NOT_ON_TABLE'
            )

        else:

            reason = (
                'OBJECT_TILTED'
            )

        self.publish_result(
            success,
            reason,
            {
                'object_x_m':
                    x,

                'object_y_m':
                    y,

                'object_z_m':
                    z,

                'xy_error_m':
                    xy_error,

                'z_error_m':
                    z_error,

                'tilt_deg':
                    math.degrees(
                        tilt
                    ),
            },
        )

    # ========================================================
    # Real robot result
    # ========================================================

    def operator_result_callback(
        self,
        message,
    ):

        if not self.pending_result:

            self.get_logger().warn(
                'Operator result ignored: '
                'no trial is awaiting evaluation.'
            )

            return

        if message.data:

            self.publish_result(
                True,
                'OPERATOR_CONFIRMED_SUCCESS',
            )

        else:

            self.publish_result(
                False,
                'OPERATOR_REPORTED_FAILURE',
            )

    # ========================================================
    # Timer
    # ========================================================

    def update(self):

        if not self.pending_result:

            return

        if not self.config.is_simulation:

            return

        if (
            self.now_seconds()
            <
            self.evaluate_after
        ):

            return

        self.evaluate_simulation()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ResultMonitor()

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
