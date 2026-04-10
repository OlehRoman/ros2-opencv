import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def find_largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


class DetectColorsNode(Node):
    def __init__(self):
        super().__init__('detect_colors_node')

        self.publisher_ = self.create_publisher(String, '/detected_objects', 10)
        
        # Використовуємо драйвер V4L2 для кращої сумісності з Docker
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # ФОРСУЄМО MJPG та роздільну здатність для стабільності в WSL2
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error('Camera not opened')
            raise RuntimeError('Camera not opened')

        self.image_width = 640
        self.image_height = 480
        self.center_x = self.image_width // 2
        self.center_y = self.image_height // 2

        self.timer = self.create_timer(0.03, self.process_frame)
    def detect_object(self, frame, mask, color_name, box_color):
        contour = find_largest_contour(mask)
        if contour is None:
            return

        area = cv2.contourArea(contour)
        if area < 500:
            return

        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        dx = cx - self.center_x
        dy = cy - self.center_y

        cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
        cv2.circle(frame, (cx, cy), 5, box_color, -1)
        cv2.line(frame, (self.center_x, 0), (self.center_x, self.image_height), (255, 255, 255), 1)
        cv2.line(frame, (0, self.center_y), (self.image_width, self.center_y), (255, 255, 255), 1)

        cv2.putText(
            frame,
            f"{color_name}: cx={cx}, cy={cy}, area={int(area)}",
            (x, y - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            box_color,
            2
        )
        cv2.putText(
            frame,
            f"dx={dx}, dy={dy}",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            box_color,
            2
        )

        msg = String()
        msg.data = f"{color_name},{cx},{cy},{int(area)},{dx},{dy}"
        self.publisher_.publish(msg)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame')
            return

        frame = cv2.resize(frame, (self.image_width, self.image_height))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])

        lower_green = np.array([35, 80, 80])
        upper_green = np.array([85, 255, 255])

        lower_blue = np.array([90, 80, 80])
        upper_blue = np.array([130, 255, 255])

        red_mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        red_mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

        self.detect_object(frame, red_mask, 'red', (0, 0, 255))
        self.detect_object(frame, green_mask, 'green', (0, 255, 0))
        self.detect_object(frame, blue_mask, 'blue', (255, 0, 0))

        cv2.imshow('detect_colors', frame)

        key = cv2.waitKey(1)
        if key == 27:
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectColorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()