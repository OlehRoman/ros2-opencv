import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mg400_msgs.action import MovL


class MG400Executor(Node):
    def __init__(self):
        super().__init__('mg400_executor')

        self.declare_parameter('command_cooldown', 1.0)
        self.declare_parameter('shift_step', 0.01)
        self.declare_parameter('approach_step', 0.02)
        self.declare_parameter('touch_z', 0.02)
        self.declare_parameter('observe_x', 0.0)
        self.declare_parameter('observe_y', 0.0)
        self.declare_parameter('observe_z', 0.20)
        self.declare_parameter('action_name', '/mg400/mov_l')

        self.command_cooldown = self.get_parameter('command_cooldown').value
        self.shift_step = self.get_parameter('shift_step').value
        self.approach_step = self.get_parameter('approach_step').value
        self.touch_z = self.get_parameter('touch_z').value
        self.observe_x = self.get_parameter('observe_x').value
        self.observe_y = self.get_parameter('observe_y').value
        self.observe_z = self.get_parameter('observe_z').value
        action_name = self.get_parameter('action_name').value

        self.subscription = self.create_subscription(
            String,
            '/touch_commands',
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(String, '/touch_status', 10)

        self.action_client = ActionClient(self, MovL, action_name)

        self.busy = False
        self.last_command_time = 0.0

        self.current_x = self.observe_x
        self.current_y = self.observe_y
        self.current_z = self.observe_z

        self.get_logger().info(
            f'Executor params: cooldown={self.command_cooldown}, shift={self.shift_step}, '
            f'approach={self.approach_step}, touch_z={self.touch_z}, '
            f'observe=({self.observe_x}, {self.observe_y}, {self.observe_z}), action={action_name}'
        )

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f'Published status: {text}')

    def command_callback(self, msg):
        if self.busy:
            self.get_logger().info('Robot is busy, skipping new command')
            return

        now = time.time()
        if now - self.last_command_time < self.command_cooldown:
            return

        parts = msg.data.split(',')
        command = parts[0]

        color = None
        if len(parts) > 1:
            color = parts[1]

        self.get_logger().info(f'Received command: {msg.data}')

        target_x = self.current_x
        target_y = self.current_y
        target_z = self.current_z

        if command == 'shift_left':
            target_x -= self.shift_step

        elif command == 'shift_right':
            target_x += self.shift_step

        elif command == 'shift_up':
            target_y += self.shift_step

        elif command == 'shift_down':
            target_y -= self.shift_step

        elif command == 'approach':
            target_z = max(self.touch_z, self.current_z - self.approach_step)

        elif command == 'touch':
            target_z = self.touch_z

        elif command == 'observe':
            target_x = self.observe_x
            target_y = self.observe_y
            target_z = self.observe_z

        else:
            self.get_logger().warning(f'Unknown command: {command}')
            return

        self.send_movl_goal(target_x, target_y, target_z, color, command)
        self.last_command_time = now

    def send_movl_goal(self, x, y, z, color, command):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('MovL action server not available')

            self.current_x = x
            self.current_y = y
            self.current_z = z

            self.get_logger().info(
                f'[SIM] {command} -> pose ({x:.3f}, {y:.3f}, {z:.3f})'
            )

            if command == 'touch' and color is not None:
                self.publish_status(f'touched,{color}')

            if command == 'observe':
                self.publish_status('observe_done')

            return

        goal_msg = MovL.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'mg400_base_link'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose
        goal_msg.set_speed_l = True
        goal_msg.speed_l = 10
        goal_msg.set_acc_l = True
        goal_msg.acc_l = 10
        goal_msg.set_cp = False
        goal_msg.cp = 0

        self.busy = True

        self.get_logger().info(
            f'Sending MovL goal: ({x:.3f}, {y:.3f}, {z:.3f})'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, x, y, z, color, command)
        )

    def goal_response_callback(self, future, x, y, z, color, command):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning('MovL goal rejected')
            self.busy = False
            return

        self.get_logger().info('MovL goal accepted')

        self.current_x = x
        self.current_y = y
        self.current_z = z

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, color, command)
        )

    def feedback_callback(self, feedback_msg):
        pose = feedback_msg.feedback.current_pose.pose.position
        self.get_logger().info(
            f'Feedback current pose: x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}'
        )

    def result_callback(self, future, color, command):
        result = future.result().result

        if result.result:
            self.get_logger().info('MovL command completed successfully')
        else:
            self.get_logger().warning('MovL command failed')

        if command == 'touch' and color is not None:
            self.publish_status(f'touched,{color}')

        if command == 'observe':
            self.publish_status('observe_done')

        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MG400Executor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()