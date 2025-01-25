#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from auv_interfaces.srv import YoloInference
from auv_interfaces.msg import Detection

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Import the ultralytics YOLO class
from ultralytics import YOLO


class YOLOv8:
    """
    YOLOv8 object detection model class for handling inference and visualization,
    using the ultralytics library with a TensorRT engine file.
    """

    def __init__(self, engine_path, confidence_thres, iou_thres, logger):
        """
        Initializes an instance of the YOLOv8 class using ultralytics.

        Args:
            engine_path (str): Path to the TensorRT engine model (e.g., 'best.engine').
            confidence_thres (float): Confidence threshold for filtering detections.
            iou_thres (float): IoU (Intersection over Union) threshold for NMS.
            logger (rclpy.logging.Logging): ROS 2 logger.
        """
        self.logger = logger

        # Initialize the ultralytics YOLO model from a TensorRT engine
        try:
            self.model = YOLO(engine_path)
            self.logger.info(f"Loaded YOLO model from {engine_path}")
        except Exception as e:
            self.logger.error(f"Failed to load the YOLO engine: {e}")
            raise e

        # Set the model configuration
        self.model.conf = confidence_thres
        self.model.iou = iou_thres

        # Move model to GPU device=0 and set half precision
        try:
            self.model.to("cuda:0")  # device=0
            self.model.half()        # half precision
            self.logger.info("Model moved to CUDA:0 with half precision.")
        except Exception as e:
            self.logger.error(f"Failed to set device/precision: {e}")
            raise e

        # Our model has only one class: "Gate"
        self.classes = ["Gate"]

        # For visualization, you can generate or fix a single color (here we do random for demonstration).
        np.random.seed(42)  # consistent color each run
        self.color_palette = np.random.uniform(0, 255, size=(1, 3))  # only 1 color

    def draw_detections(self, img, box, score):
        """
        Draws bounding boxes and labels on the input image for the single class 'Gate'.

        Args:
            img (numpy.ndarray): The input image to draw on.
            box (list): Detected bounding box [x1, y1, x2, y2].
            score (float): Detection confidence score.
        """
        x1, y1, x2, y2 = box
        # Single color for 'Gate'
        color = self.color_palette[0]

        # Draw bounding box
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

        label = f"{self.classes[0]}: {score:.2f}"
        (label_width, label_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        label_x = x1
        label_y = y1 - 10 if y1 - 10 > label_height else y1 + 10

        # Rectangle behind label text
        cv2.rectangle(
            img,
            (int(label_x), int(label_y - label_height)),
            (int(label_x + label_width), int(label_y)),
            color,
            cv2.FILLED,
        )
        # Label text
        cv2.putText(
            img,
            label,
            (int(label_x), int(label_y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )

    def perform_inference(self, cv_image):
        """
        Performs inference using the YOLOv8 model from ultralytics.

        Args:
            cv_image (numpy.ndarray): The input image in OpenCV BGR format.

        Returns:
            tuple: (annotated_image, list_of_detections)
                   annotated_image (numpy.ndarray): The image with bounding boxes drawn.
                   list_of_detections (list): List of auv_interfaces.msg.Detection objects.
        """
        # Run inference
        results = self.model.predict(
            source=cv_image,
            conf=self.model.conf,
            iou=self.model.iou
        )

        annotated_image = cv_image.copy()
        detections = []

        # Each 'results' can have .boxes which contain xyxy, confidence, class
        for result in results:  # Typically just 1 result for a single image
            if not hasattr(result, 'boxes'):
                continue
            boxes_data = result.boxes  # A Boxes object from ultralytics

            for box in boxes_data:
                # box.xyxy, box.conf, box.cls
                xyxy = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                conf = float(box.conf[0].cpu().numpy())
                # Our single class is always index 0 if there's a detection
                cls_id = int(box.cls[0].cpu().numpy())

                # Convert it to our custom Detection message format
                detection_msg = Detection()
                detection_msg.object = self.classes[0]
                detection_msg.confidence = conf
                detection_msg.x1 = int(xyxy[0])
                detection_msg.y1 = int(xyxy[1])
                detection_msg.x2 = int(xyxy[2])
                detection_msg.y2 = int(xyxy[3])

                detections.append(detection_msg)

                # Draw the detection
                self.draw_detections(
                    annotated_image,
                    [xyxy[0], xyxy[1], xyxy[2], xyxy[3]],
                    conf
                )

        return annotated_image, detections


class YoloInferenceServer(Node):
    """ROS 2 Service Server for YOLOv8 Inference using ultralytics and TensorRT engine."""

    def __init__(self):
        super().__init__("yolo_inference_server")

        # Initialize service
        self.srv = self.create_service(
            YoloInference, "yolo_inference", self.handle_inference
        )

        # Path to your TensorRT engine file, e.g. best.engine
        model_path = get_package_share_directory("auv_ml") + "/models/best.engine"

        # Initialize YOLOv8 instance (single-class: Gate) with device=0 (cuda:0), half=True
        self.yolo = YOLOv8(
            engine_path=model_path,
            confidence_thres=0.25,  # Adjust if needed
            iou_thres=0.5,         # Adjust if needed
            logger=self.get_logger(),
        )

        self.get_logger().info("YOLOv8 Inference Server (Single-Class: Gate) is ready.")

    def handle_inference(self, request, response):
        """
        Handles incoming inference requests.

        Args:
            request (YoloInference.Request): The service request containing the image.
            response (YoloInference.Response): The service response to be filled.

        Returns:
            YoloInference.Response: The response containing the annotated image and detections.
        """
        bridge = CvBridge()

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            response.success = False
            response.error_message = f"CvBridge Error: {e}"
            response.result_image = request.image  # Return original image
            response.detections = []
            return response

        # Perform inference
        try:
            annotated_image, detections = self.yolo.perform_inference(cv_image)
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")
            response.success = False
            response.error_message = f"Inference Error: {e}"
            response.result_image = request.image  # Return original image
            response.detections = []
            return response

        # Convert annotated image back to ROS Image message
        try:
            result_image_msg = bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            response.success = False
            response.error_message = f"CvBridge Error: {e}"
            response.result_image = request.image  # Return original image
            response.detections = []
            return response

        # Populate response
        response.result_image = result_image_msg
        response.detections = detections
        response.success = True
        response.error_message = ""

        self.get_logger().info(
            f"Inference completed with {len(detections)} detections."
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    server = YoloInferenceServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
