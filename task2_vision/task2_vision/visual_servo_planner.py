import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisualServoPlanner(Node):
    def __init__(self):
        super().__init__('visual_servo_planner')

        self.detect_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.detect_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/touch_status',
            self.status_callback,
            10
        )

        self.command_pub = self.create_publisher(String, '/touch_commands', 10)

        self.objects = {}
        self.object_timeout = 1.0
        self.processed_objects = set()
        self.current_target = None

        self.last_command_time = 0.0
        self.command_cooldown = 1.0

        self.center_tolerance_x = 40
        self.center_tolerance_y = 40
        self.touch_area_threshold = 18000

        self.state = 'SEARCHING'

        self.timer = self.create_timer(0.5, self.control_loop)

    def detect_callback(self, msg):
        try:
            color, cx, cy, area, dx, dy = msg.data.split(',')
            self.objects[color] = {
                'cx': int(cx),
                'cy': int(cy),
                'area': int(area),
                'dx': int(dx),
                'dy': int(dy),
                'last_seen': time.time()
            }
        except Exception as e:
            self.get_logger().warning(f'Failed to parse detection: {msg.data}, error: {e}')

    def status_callback(self, msg):
        data = msg.data.strip()

        if data.startswith('touched,'):
            _, color = data.split(',', 1)
            self.get_logger().info(f'Touch confirmed for {color}')
            self.processed_objects.add(color)
            self.current_target = None
            self.state = 'RETURNING'
            self.publish_command('observe')

        elif data == 'observe_done':
            self.get_logger().info('Observe pose reached')
            self.state = 'SEARCHING'

    def remove_stale_objects(self):
        now = time.time()
        stale = []

        for color, data in self.objects.items():
            if now - data['last_seen'] > self.object_timeout:
                stale.append(color)

        for color in stale:
            self.get_logger().info(f'Removing stale target: {color}')
            del self.objects[color]
            if self.current_target == color:
                self.current_target = None
                if self.state == 'ALIGNING':
                    self.state = 'SEARCHING'

    def choose_target(self):
        for color in ['red', 'green', 'blue']:
            if color in self.objects and color not in self.processed_objects:
                return color
        return None

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.last_command_time = time.time()
        self.get_logger().info(f'Published command: {command}')

    def control_loop(self):
        self.remove_stale_objects()

        available = [c for c in self.objects if c not in self.processed_objects]

        if len(self.processed_objects) >= 3:
            self.get_logger().info('All three targets processed')
            return

        if self.state == 'RETURNING':
            return

        if not available:
            self.get_logger().info('No unprocessed targets visible')
            self.current_target = None
            return

        if self.current_target is None:
            self.current_target = self.choose_target()
            if self.current_target:
                self.get_logger().info(f'Selected target: {self.current_target}')
                self.state = 'ALIGNING'
            return

        if self.current_target not in self.objects:
            self.current_target = None
            self.state = 'SEARCHING'
            return

        now = time.time()
        if now - self.last_command_time < self.command_cooldown:
            return

        data = self.objects[self.current_target]
        dx = data['dx']
        dy = data['dy']
        area = data['area']

        if self.state != 'ALIGNING':
            return

        if abs(dx) > self.center_tolerance_x:
            if dx > 0:
                self.publish_command(f'shift_right,{self.current_target}')
            else:
                self.publish_command(f'shift_left,{self.current_target}')
            return

        if abs(dy) > self.center_tolerance_y:
            if dy > 0:
                self.publish_command(f'shift_down,{self.current_target}')
            else:
                self.publish_command(f'shift_up,{self.current_target}')
            return

        if area < self.touch_area_threshold:
            self.publish_command(f'approach,{self.current_target}')
            return

        self.publish_command(f'touch,{self.current_target}')


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()