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

from ultralytics import YOLO
from sort import SORT

class YOLOv8:
    """
    YOLOv8 model class for Gate detection using a TensorRT engine with SORT tracking.
    """
    def __init__(self, engine_path, confidence_thres, iou_thres, logger):
        self.logger = logger
        try:
            self.model = YOLO(engine_path)
            self.logger.info(f"Loaded YOLO model from {engine_path}")
        except Exception as e:
            self.logger.error(f"Failed to load YOLO engine: {e}")
            raise e

        self.model.conf = confidence_thres
        self.model.iou = iou_thres
        self.classes = ["Gate"]
        np.random.seed(42)
        self.color_palette = np.random.uniform(0, 255, size=(1, 3))
        self.tracker = SORT(max_age=5, min_hits=3, iou_threshold=0.3)

    def draw_detections(self, img, box, score):
        x1, y1, x2, y2 = box
        color = self.color_palette[0]
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        label = f"{self.classes[0]}: {score:.2f}"
        (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        label_x = x1
        label_y = y1 - 10 if y1 - 10 > label_height else y1 + 10
        cv2.rectangle(img,
                      (int(label_x), int(label_y - label_height)),
                      (int(label_x + label_width), int(label_y)),
                      color, cv2.FILLED)
        cv2.putText(img, label, (int(label_x), int(label_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def perform_inference(self, cv_image):
        results = self.model.predict(
            source=cv_image,
            conf=self.model.conf,
            iou=self.model.iou,
            device=0,
            half=True
        )
        annotated_image = cv_image.copy()
        detections = []
        detection_boxes = []  # Each detection: [xmin, ymin, xmax, ymax, score]

        for result in results:
            if not hasattr(result, 'boxes'):
                continue
            boxes_data = result.boxes
            for box in boxes_data:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                detection_boxes.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])
                detection_msg = Detection()
                detection_msg.object = self.classes[0]
                detection_msg.confidence = conf
                detection_msg.x1 = int(xyxy[0])
                detection_msg.y1 = int(xyxy[1])
                detection_msg.x2 = int(xyxy[2])
                detection_msg.y2 = int(xyxy[3])
                detections.append(detection_msg)
                self.draw_detections(annotated_image, [xyxy[0], xyxy[1], xyxy[2], xyxy[3]], conf)

        if len(detection_boxes) > 0:
            boxes_for_tracker = []
            for box in detection_boxes:
                x1, y1, x2, y2, score = box
                w = x2 - x1
                h = y2 - y1
                boxes_for_tracker.append([x1, y1, w, h])
            boxes_for_tracker = np.array(boxes_for_tracker)
            self.tracker.run(boxes_for_tracker, 0)
            tracks = self.tracker.get_tracks(0)
            for track in tracks:
                track_id, tx, ty, tw, th = track
                cv2.putText(annotated_image, f"ID:{int(track_id)}",
                            (int(tx), int(ty) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self.tracker.run(np.empty((0, 4)), 0)

        return annotated_image, detections

class YoloInferenceServer(Node):
    def __init__(self):
        super().__init__("yolo_inference_server")
        self.srv = self.create_service(YoloInference, "yolo_inference_server", self.handle_inference)
        model_path = get_package_share_directory("auv_ml") + "/models/gate-newest.engine"
        self.yolo = YOLOv8(engine_path=model_path,
                           confidence_thres=0.4,
                           iou_thres=0.5,
                           logger=self.get_logger())
        self.get_logger().info("YOLOv8 Inference Server (Gate with SORT tracking) is ready.")

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

        try:
            annotated_image, detections = self.yolo.perform_inference(cv_image)
        except Exception as e:
            self.get_logger().error(f"Inference Error: {e}")
            response.success = False
            response.error_message = f"Inference Error: {e}"
            return response

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
    server = YoloInferenceServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
