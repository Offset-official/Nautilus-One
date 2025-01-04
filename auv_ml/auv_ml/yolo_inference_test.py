#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auv_interfaces.srv import YoloInference
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os


class YoloInferenceClient(Node):
    """ROS 2 Service Client for YOLOv8 Inference."""

    def __init__(self):
        super().__init__('yolo_inference_test')
        self.cli = self.create_client(YoloInference, 'yolo_inference_server')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.req = YoloInference.Request()
        self.bridge = CvBridge()

    def send_request(self, input_image_path, output_image_path):
        """
        Sends a request to the YOLO inference service.

        Args:
            input_image_path (str): Path to the input image.
            output_image_path (str): Path to save the annotated output image.

        Returns:
            None
        """
        # Read the input image
        if not os.path.exists(input_image_path):
            self.get_logger().error(f"Input image does not exist: {input_image_path}")
            return

        cv_image = cv2.imread(input_image_path)
        if cv_image is None:
            self.get_logger().error(f"Failed to read image: {input_image_path}")
            return

        # Convert OpenCV image to ROS Image message
        try:
            self.req.image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Call the service
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            response = self.future.result()
            if response.success:
                # Convert ROS Image message back to OpenCV image
                try:
                    annotated_image = self.bridge.imgmsg_to_cv2(
                        response.result_image, desired_encoding="bgr8"
                    )
                    # Save the annotated image
                    cv2.imwrite(output_image_path, annotated_image)
                    self.get_logger().info(
                        f"Annotated image saved to {output_image_path}"
                    )
                except CvBridgeError as e:
                    self.get_logger().error(f"CvBridge Error: {e}")
            else:
                self.get_logger().error(f"Service failed: {response.error_message}")
        else:
            self.get_logger().error("Service call failed")


def main(args=None):
    if len(sys.argv) != 3:
        print(
            "[red]Usage: yolo_inference_test.py <input_image_path> <output_image_path>[/red]"
        )
        sys.exit(1)

    input_image_path = sys.argv[1]
    output_image_path = sys.argv[2]

    rclpy.init(args=args)

    client = YoloInferenceClient()
    client.send_request(input_image_path, output_image_path)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
