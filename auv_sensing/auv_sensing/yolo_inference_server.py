#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from auv_interfaces.srv import YoloInference
from auv_interfaces.msg import Detection
import cv2
import numpy as np
import onnxruntime as ort
from cv_bridge import CvBridge, CvBridgeError

class YOLOv8:
    """YOLOv8 object detection model class for handling inference and visualization."""

    def __init__(self, onnx_model, confidence_thres, iou_thres, logger):
        """
        Initializes an instance of the YOLOv8 class.

        Args:
            onnx_model (str): Path to the ONNX model.
            confidence_thres (float): Confidence threshold for filtering detections.
            iou_thres (float): IoU (Intersection over Union) threshold for non-maximum suppression.
            logger (rclpy.logging.Logging): ROS 2 logger.
        """
        self.onnx_model = onnx_model
        self.confidence_thres = confidence_thres
        self.iou_thres = iou_thres
        self.logger = logger

        self.classes = [
            "Blue flare", "Blue pail", "Cloth", "Gate", "Red flare", "Red pail",
            "Yellow flare"
        ]

        # Generate a color palette for the classes
        np.random.seed(42)  # For reproducibility
        self.color_palette = np.random.uniform(0, 255, size=(len(self.classes), 3))

        # Initialize ONNX Runtime session
        try:
            self.session = ort.InferenceSession(self.onnx_model, providers=["CPUExecutionProvider"])
            self.logger.info("ONNX Runtime session successfully created.")
        except Exception as e:
            self.logger.error(f"Failed to create ONNX Runtime session: {e}")
            raise e

        # Get model input details
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.input_width = self.input_shape[3]
        self.input_height = self.input_shape[2]
        self.logger.info(f"Model input name: {self.input_name}")
        self.logger.info(f"Model input dimensions: {self.input_width} x {self.input_height}")

        # Initialize CvBridge
        self.bridge = CvBridge()

    def draw_detections(self, img, box, score, class_id):
        """
        Draws bounding boxes and labels on the input image based on the detected objects.

        Args:
            img (numpy.ndarray): The input image to draw detections on.
            box (list): Detected bounding box [x1, y1, w, h].
            score (float): Detection confidence score.
            class_id (int): Class ID for the detected object.

        Returns:
            None
        """
        # Extract the coordinates of the bounding box
        x1, y1, w, h = box

        # Retrieve the color for the class ID
        color = self.color_palette[class_id]

        # Draw the bounding box on the image
        cv2.rectangle(img, (int(x1), int(y1)), (int(x1 + w), int(y1 + h)), color, 2)

        # Create the label text with class name and score
        label = f"{self.classes[class_id]}: {score:.2f}"

        # Calculate the dimensions of the label text
        (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

        # Calculate the position of the label text
        label_x = x1
        label_y = y1 - 10 if y1 - 10 > label_height else y1 + 10

        # Draw a filled rectangle as the background for the label text
        cv2.rectangle(
            img, (int(label_x), int(label_y - label_height)), 
            (int(label_x + label_width), int(label_y + 0)), 
            color, cv2.FILLED
        )

        # Draw the label text on the image
        cv2.putText(img, label, (int(label_x), int(label_y)), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

    def preprocess(self, cv_image):
        """
        Preprocesses the input image before performing inference.

        Args:
            cv_image (numpy.ndarray): The input image in OpenCV format.

        Returns:
            numpy.ndarray: Preprocessed image data ready for inference.
        """
        # Get the height and width of the input image
        img_height, img_width = cv_image.shape[:2]

        # Convert the image color space from BGR to RGB
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Resize the image to match the input shape
        img = cv2.resize(img, (self.input_width, self.input_height))

        # Normalize the image data by dividing it by 255.0
        image_data = np.array(img) / 255.0

        # Transpose the image to have the channel dimension as the first dimension
        image_data = np.transpose(image_data, (2, 0, 1))  # Channel first

        # Expand the dimensions of the image data to match the expected input shape
        image_data = np.expand_dims(image_data, axis=0).astype(np.float32)

        return image_data

    def postprocess(self, input_image, output):
        """
        Performs post-processing on the model's output to extract bounding boxes, scores, and class IDs.

        Args:
            input_image (numpy.ndarray): The input image.
            output (list): The output of the model.

        Returns:
            tuple: The input image with detections drawn on it and a list of detections.
        """
        # Transpose and squeeze the output to match the expected shape
        outputs = np.transpose(np.squeeze(output[0]))

        # Get the number of rows in the outputs array
        rows = outputs.shape[0]

        # Lists to store the bounding boxes, scores, and class IDs of the detections
        boxes = []
        scores = []
        class_ids = []

        # Calculate the scaling factors for the bounding box coordinates
        x_factor = input_image.shape[1] / self.input_width
        y_factor = input_image.shape[0] / self.input_height

        # Iterate over each row in the outputs array
        for i in range(rows):
            # Extract the class scores from the current row
            classes_scores = outputs[i][4:]

            # Find the maximum score among the class scores
            max_score = np.amax(classes_scores)

            # If the maximum score is above the confidence threshold
            if max_score >= self.confidence_thres:
                # Get the class ID with the highest score
                class_id = np.argmax(classes_scores)

                # Extract the bounding box coordinates from the current row
                x, y, w, h = outputs[i][0], outputs[i][1], outputs[i][2], outputs[i][3]

                # Calculate the scaled coordinates of the bounding box
                left = int((x - w / 2) * x_factor)
                top = int((y - h / 2) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)

                # Clamp coordinates to image boundaries
                left = max(0, min(left, input_image.shape[1] - 1))
                top = max(0, min(top, input_image.shape[0] - 1))
                width = max(0, min(width, input_image.shape[1] - left))
                height = max(0, min(height, input_image.shape[0] - top))

                boxes.append([left, top, width, height])
                scores.append(max_score)
                class_ids.append(class_id)

        # Apply non-maximum suppression to filter out overlapping bounding boxes
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_thres, self.iou_thres)

        detections = []
        # Iterate over the selected indices after non-maximum suppression
        for i in indices.flatten():
            box = boxes[i]
            score = scores[i]
            class_id = class_ids[i]

            # Create detection dictionary
            detection = Detection()
            detection.object = self.classes[class_id]
            detection.confidence = float(score)
            detection.x1 = int(box[0])
            detection.y1 = int(box[1])
            detection.x2 = int(box[0] + box[2])
            detection.y2 = int(box[1] + box[3])

            detections.append(detection)

            # Draw the detection on the input image
            self.draw_detections(input_image, box, score, class_id)

        return input_image, detections

    def perform_inference(self, cv_image):
        """
        Performs inference using the YOLOv8 model.

        Args:
            cv_image (numpy.ndarray): The input image in OpenCV format.

        Returns:
            tuple: The annotated image and a list of detections.
        """
        # Preprocess the image
        img_data = self.preprocess(cv_image)

        # Run inference
        try:
            outputs = self.session.run(None, {self.input_name: img_data})
        except Exception as e:
            self.logger.error(f"Inference failed: {e}")
            raise e

        # Postprocess the outputs
        annotated_image, detections = self.postprocess(cv_image, outputs)

        return annotated_image, detections

class YoloInferenceServer(Node):
    """ROS 2 Service Server for YOLOv8 Inference."""

    def __init__(self):
        super().__init__('yolo_inference_server')

        # Initialize service
        self.srv = self.create_service(YoloInference, 'yolo_inference', self.handle_inference)

        # Get model path
        model_path = get_package_share_directory('auv_sensing') + '/models/best.onnx'

        # Initialize YOLOv8 instance
        self.yolo = YOLOv8(
            onnx_model=model_path,
            confidence_thres=0.5,
            iou_thres=0.5,
            logger=self.get_logger()
        )

        self.get_logger().info('YOLOv8 Inference Server is ready.')

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
            cv_image = bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
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
            result_image_msg = bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
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

        self.get_logger().info(f"Inference completed with {len(detections)} detections.")

        return response

def main(args=None):
    rclpy.init(args=args)
    server = YoloInferenceServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
