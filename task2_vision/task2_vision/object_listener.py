import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ObjectListenerNode(Node):
    def __init__(self):
        super().__init__('object_listener_node')

        self.subscription = self.create_subscription(
            String,
            '/detected_objects',
            self.listener_callback,
            10
        )

        self.objects = {}

    def listener_callback(self, msg):
        try:
            color, x, y = msg.data.split(',')
            x = int(x)
            y = int(y)

            self.objects[color] = (x, y)

            self.get_logger().info(
                f"Detected objects: {self.objects}"
            )
        except Exception as e:
            self.get_logger().warning(f"Failed to parse message: {msg.data}, error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()