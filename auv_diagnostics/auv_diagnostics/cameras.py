import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/usb_cam_0/image_raw/compressed',
            self.image_callback,
            10)
        self.subscription

        cv2.namedWindow('Camera Feed', cv2.WINDOW_AUTOSIZE)

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image_np is not None:
            cv2.imshow('Camera Feed', image_np)
            cv2.waitKey(1)
        else:
            self.get_logger().info('Image is None')

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


if __name__ == '__main__':
    main()
