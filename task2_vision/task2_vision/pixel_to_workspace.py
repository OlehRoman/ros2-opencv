import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PixelToWorkspaceNode(Node):
    def __init__(self):
        super().__init__('pixel_to_workspace_node')

        self.subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.callback,
            10
        )

        self.publisher_ = self.create_publisher(String, '/workspace_targets', 10)

        # Параметри кадру. Потім можна уточнити під твою камеру.
        self.image_width = 640
        self.image_height = 480

        # Умовний розмір робочої області робота в метрах.
        self.workspace_x_min = -0.15
        self.workspace_x_max = 0.15
        self.workspace_y_min = -0.10
        self.workspace_y_max = 0.10

    def map_value(self, value, in_min, in_max, out_min, out_max):
        return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

    def callback(self, msg):
        try:
            color, x, y = msg.data.split(',')
            x = int(x)
            y = int(y)

            wx = self.map_value(
                x,
                0,
                self.image_width,
                self.workspace_x_min,
                self.workspace_x_max
            )

            wy = self.map_value(
                y,
                0,
                self.image_height,
                self.workspace_y_max,
                self.workspace_y_min
            )

            out_msg = String()
            out_msg.data = f'{color},{wx:.3f},{wy:.3f}'
            self.publisher_.publish(out_msg)

            self.get_logger().info(
                f'Pixel ({x}, {y}) -> Workspace ({wx:.3f}, {wy:.3f}) for {color}'
            )

        except Exception as e:
            self.get_logger().warning(f'Failed to convert message: {msg.data}, error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PixelToWorkspaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()