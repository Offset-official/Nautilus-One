import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__("compressed_image_subscriber")

        # Subscriptions for two cameras
        self.subscription1 = self.create_subscription(
            CompressedImage,
            "/usb_cam_0/image_raw/compressed",
            self.image_callback_cam0,
            10,
        )

        self.subscription2 = self.create_subscription(
            CompressedImage,
            "/usb_cam_1/image_raw/compressed",
            self.image_callback_cam1,
            10,
        )

        self.subscription3 = self.create_subscription(
            CompressedImage,
            "/usb_cam_2/image_raw/compressed",
            self.image_callback_cam2,
            10,
        )

        # Create two windows for displaying camera feeds
        cv2.namedWindow("Camera 0 Feed", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Camera 1 Feed", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Camera 2 Feed", cv2.WINDOW_AUTOSIZE)

    def image_callback_cam0(self, msg):
        self._process_image(msg, "Camera 0 Feed")

    def image_callback_cam1(self, msg):
        self._process_image(msg, "Camera 1 Feed")

    def image_callback_cam2(self, msg):
        self._process_image(msg, "Camera 2 Feed")

    def _process_image(self, msg, window_name):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image_np is not None:
            cv2.imshow(window_name, image_np)
            cv2.waitKey(1)
        else:
            self.get_logger().info(f"Image from {window_name} is None")

    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    try:
        compressed_image_subscriber = CompressedImageSubscriber()
        rclpy.spin(compressed_image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        compressed_image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
