#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import DetectionArray

class GateDetectionAreaNode(Node):
    def __init__(self):
        super().__init__('gate_detection_area_node')
        # Subscribe to the detections topic
        self.subscription = self.create_subscription(
            DetectionArray,
            '/auv_camera_front/detections',
            self.listener_callback,
            10)
        # Total image area (1280 x 720 pixels)
        self.image_area = 1280 * 720

    def listener_callback(self, msg):
        # Iterate over each detection in the array
        for detection in msg.detections:
            if detection.object == "Gate":
                # Compute width and height of the bounding box in pixels
                width = detection.x2 - detection.x1
                height = detection.y2 - detection.y1
                if width < 0 or height < 0:
                    self.get_logger().warn(f"Invalid bounding box for detection: {detection}")
                    continue
                # Compute area of the bounding box
                box_area = width * height
                # Calculate the ratio with respect to the total image area
                ratio = box_area / self.image_area
                self.get_logger().info(f"Gate detection ratio: {ratio:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = GateDetectionAreaNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
