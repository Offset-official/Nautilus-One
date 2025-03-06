#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class CompressedToImageNode(Node):
    def __init__(self):
        super().__init__('compressed_to_image_node')
        
        # Declare and get parameters for topic names
        self.declare_parameter('input_topic', '/camera/image/compressed')
        self.declare_parameter('output_topic', '/camera/image')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.get_logger().info(f"Subscribing to compressed topic: {input_topic}")
        self.get_logger().info(f"Publishing decompressed images to: {output_topic}")
        
        # Create subscriber for the compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.callback,
            10)
        
        # Create publisher for the normal image topic
        self.publisher = self.create_publisher(Image, output_topic, 10)
        
        # CvBridge for converting between ROS messages and OpenCV images
        self.bridge = CvBridge()

    def callback(self, msg: CompressedImage):
        try:
            # Convert the compressed image data to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            # Decode the image using OpenCV
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Error decoding compressed image: {e}")
            return

        try:
            # Convert the OpenCV image to a ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(image_np, "bgr8")
            self.publisher.publish(image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedToImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
