import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Task2Controller(Node):
    def __init__(self):
        super().__init__('task2_controller')

        self.subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.callback,
            10
        )

        self.objects = {}
        self.current_target = None
        self.object_timeout = 1.0  # seconds

        self.timer = self.create_timer(1.0, self.control_loop)

    def callback(self, msg):
        try:
            color, x, y = msg.data.split(',')
            self.objects[color] = {
                'x': int(x),
                'y': int(y),
                'last_seen': time.time()
            }
        except Exception as e:
            self.get_logger().warning(f'Failed to parse message: {msg.data}, error: {e}')

    def remove_stale_objects(self):
        now = time.time()
        stale_colors = []

        for color, data in self.objects.items():
            if now - data['last_seen'] > self.object_timeout:
                stale_colors.append(color)

        for color in stale_colors:
            self.get_logger().info(f'Removing stale object: {color}')
            del self.objects[color]

            if self.current_target == color:
                self.get_logger().info(f'Lost current target: {color}')
                self.current_target = None

    def choose_target(self):
        for color in ['red', 'green', 'blue']:
            if color in self.objects:
                return color
        return None

    def control_loop(self):
        self.remove_stale_objects()

        if not self.objects:
            self.get_logger().info('No objects detected')
            self.current_target = None
            return

        if self.current_target is None:
            self.current_target = self.choose_target()
            if self.current_target:
                self.get_logger().info(f'Target selected: {self.current_target}')
            return

        if self.current_target not in self.objects:
            self.get_logger().info(f'Current target disappeared: {self.current_target}')
            self.current_target = None
            return

        data = self.objects[self.current_target]
        x = data['x']
        y = data['y']

        self.get_logger().info(
            f'Tracking {self.current_target} at ({x}, {y})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Task2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()