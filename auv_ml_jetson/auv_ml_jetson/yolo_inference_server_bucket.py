#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from auv_interfaces.srv import YoloInference
from auv_interfaces.msg import Detection

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import requests
import base64

class BucketInferenceClient(Node):
    def __init__(self):
        super().__init__("yolo_inference_client_bucket")
        self.srv = self.create_service(YoloInference, "yolo_inference_bucket", self.handle_inference)
        # Set the FastAPI server URL for bucket inference (replace <FASTAPI_SERVER_IP> with actual IP)
        self.fastapi_url = "http://192.168.2.4:8000/inference/bucket"
        self.get_logger().info(f"ROS Service Node for Bucket Inference is ready. FastAPI URL: {self.fastapi_url}")

    def handle_inference(self, request, response):
        start_time = time.time()
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            response.success = False
            response.error_message = f"CvBridge Error: {e}"
            return response

        # Encode image to JPEG and then to base64
        ret, buffer = cv2.imencode('.jpg', cv_image)
        if not ret:
            self.get_logger().error("Failed to encode image")
            response.success = False
            response.error_message = "Failed to encode image"
            return response
        img_base64 = base64.b64encode(buffer).decode('utf-8')

        payload = {"image": img_base64}
        try:
            r = requests.post(self.fastapi_url, json=payload, timeout=10)
            if r.status_code != 200:
                self.get_logger().error(f"FastAPI server error: {r.text}")
                response.success = False
                response.error_message = f"FastAPI server error: {r.text}"
                return response
            result = r.json()
        except Exception as e:
            self.get_logger().error(f"HTTP Request failed: {e}")
            response.success = False
            response.error_message = f"HTTP Request failed: {e}"
            return response

        # Decode annotated image from base64
        annotated_image_b64 = result.get("annotated_image")
        if annotated_image_b64 is None:
            self.get_logger().error("No annotated_image in response")
            response.success = False
            response.error_message = "No annotated_image in response"
            return response

        try:
            annotated_image_bytes = base64.b64decode(annotated_image_b64)
            np_arr = np.frombuffer(annotated_image_bytes, np.uint8)
            annotated_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode annotated image: {e}")
            response.success = False
            response.error_message = f"Failed to decode annotated image: {e}"
            return response

        # Process detections
        detections_data = result.get("detections", [])
        detections = []
        for det in detections_data:
            detection_msg = Detection()
            detection_msg.object = det.get("object", "")
            detection_msg.confidence = det.get("confidence", 0.0)
            detection_msg.x1 = int(det.get("x1", 0))
            detection_msg.y1 = int(det.get("y1", 0))
            detection_msg.x2 = int(det.get("x2", 0))
            detection_msg.y2 = int(det.get("y2", 0))
            detections.append(detection_msg)

        elapsed_time = time.time() - start_time
        self.get_logger().info(f"Inference Time: {elapsed_time:.3f} sec, Detections: {len(detections)}")

        try:
            result_image_msg = bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            response.success = False
            response.error_message = f"CvBridge Error: {e}"
            return response

        response.result_image = result_image_msg
        response.detections = detections
        response.success = True
        response.error_message = ""
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BucketInferenceClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
