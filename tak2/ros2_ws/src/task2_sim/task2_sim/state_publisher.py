import rclpy
from rclpy.executors import ExternalShutdownException

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from task2_sim.runtime_config import Task2Config


class Task2StatePublisher(Node):
    """Publish a unified robot state and an explicit provenance marker."""

    def __init__(self):
        super().__init__('task2_state_publisher')

        config_dir = get_package_share_directory('task2_sim') + '/config'
        self.config = Task2Config(config_dir)
        common = self.config.communication['common']
        backend = (
            self.config.communication['simulation']
            if self.config.is_simulation
            else self.config.communication['real']
        )

        self.source_joint_state_topic = backend['joint_state_topic']
        self.joint_names = [
            'joint1_to_base',
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
        ]
        self.commanded_positions = [0.0] * 6
        self.measured_positions = None
        self.last_feedback_time = None
        self.last_feedback_stamp = None
        self.feedback_timeout_s = (
            0.50
            if self.config.is_simulation
            else float(
                self.config.device.get('real', {}).get(
                    'real_feedback_max_age_s',
                    0.75,
                )
            )
        )

        self.robot_state_pub = self.create_publisher(
            JointState,
            common['robot_state_topic'],
            10,
        )
        self.source_pub = self.create_publisher(
            String,
            common['robot_state_source_topic'],
            10,
        )
        self.command_sub = self.create_subscription(
            Float64MultiArray,
            common['arm_command_topic'],
            self.command_callback,
            10,
        )
        self.feedback_sub = self.create_subscription(
            JointState,
            self.source_joint_state_topic,
            self.feedback_callback,
            10,
        )

        self.timer = self.create_timer(0.10, self.publish_robot_state)
        self.get_logger().info('State publisher started.')
        self.get_logger().info(f'Mode: {self.config.mode_name()}')
        self.get_logger().info(
            f'Feedback topic: {self.source_joint_state_topic}'
        )

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def command_callback(self, message):
        if len(message.data) != 6:
            return
        self.commanded_positions = [float(value) for value in message.data]

    def feedback_callback(self, message):
        if len(message.position) < 6:
            return

        if len(message.name) == len(message.position):
            lookup = {
                name: position
                for name, position in zip(message.name, message.position)
            }
            if all(name in lookup for name in self.joint_names):
                self.measured_positions = [
                    float(lookup[name]) for name in self.joint_names
                ]
            else:
                self.measured_positions = [
                    float(value) for value in message.position[:6]
                ]
        else:
            self.measured_positions = [
                float(value) for value in message.position[:6]
            ]

        self.last_feedback_time = self.now_seconds()
        self.last_feedback_stamp = message.header.stamp

    def publish_robot_state(self):
        now = self.now_seconds()
        feedback_valid = (
            self.measured_positions is not None
            and self.last_feedback_time is not None
            and now - self.last_feedback_time <= self.feedback_timeout_s
        )

        if feedback_valid:
            positions = list(self.measured_positions)
            source = 'MEASURED'
        else:
            positions = list(self.commanded_positions)
            source = 'COMMAND_FALLBACK'

        state = JointState()
        if feedback_valid and self.last_feedback_stamp is not None:
            state.header.stamp = self.last_feedback_stamp
        else:
            state.header.stamp = self.get_clock().now().to_msg()
        # The source topic is retained for compatibility. frame_id duplicates
        # the provenance atomically on the JointState itself so cross-topic
        # scheduling cannot make COMMAND_FALLBACK look like MEASURED feedback.
        state.header.frame_id = source
        state.name = list(self.joint_names)
        state.position = positions

        source_message = String()
        source_message.data = source
        self.source_pub.publish(source_message)
        self.robot_state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = Task2StatePublisher()

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
