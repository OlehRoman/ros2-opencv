import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TouchCommandPlanner(Node):
    def __init__(self):
        super().__init__('touch_command_planner')

        self.subscription = self.create_subscription(
            String,
            '/workspace_targets',
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(String, '/touch_commands', 10)

        self.objects = {}
        self.object_timeout = 1.0
        self.current_target = None
        self.processed_objects = set()

        self.last_published_time = 0.0
        self.publish_cooldown = 3.0

        self.timer = self.create_timer(1.0, self.control_loop)

    def callback(self, msg):
        try:
            color, x, y = msg.data.split(',')
            self.objects[color] = {
                'x': float(x),
                'y': float(y),
                'last_seen': time.time()
            }
        except Exception as e:
            self.get_logger().warning(f'Failed to parse workspace target: {msg.data}, error: {e}')

    def remove_stale_objects(self):
        now = time.time()
        stale = []

        for color, data in self.objects.items():
            if now - data['last_seen'] > self.object_timeout:
                stale.append(color)

        for color in stale:
            self.get_logger().info(f'Removing stale workspace target: {color}')
            del self.objects[color]
            if self.current_target == color:
                self.current_target = None

    def choose_target(self):
        for color in ['red', 'green', 'blue']:
            if color in self.objects and color not in self.processed_objects:
                return color
        return None

    def control_loop(self):
        self.remove_stale_objects()

        available = [c for c in self.objects if c not in self.processed_objects]

        if not available:
            self.get_logger().info('No unprocessed workspace targets available')
            self.current_target = None
            return

        if self.current_target is None:
            self.current_target = self.choose_target()
            if self.current_target:
                self.get_logger().info(f'Selected target for touch: {self.current_target}')

        if self.current_target is None:
            return

        if self.current_target not in self.objects:
            self.current_target = None
            return

        now = time.time()
        if now - self.last_published_time < self.publish_cooldown:
            return

        x = self.objects[self.current_target]['x']
        y = self.objects[self.current_target]['y']
        z = 0.02

        cmd = String()
        cmd.data = f'touch,{self.current_target},{x:.3f},{y:.3f},{z:.3f}'
        self.publisher_.publish(cmd)

        self.get_logger().info(f'Publishing touch command: {cmd.data}')

        self.processed_objects.add(self.current_target)
        self.last_published_time = now
        self.current_target = None


def main(args=None):
    rclpy.init(args=args)
    node = TouchCommandPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()