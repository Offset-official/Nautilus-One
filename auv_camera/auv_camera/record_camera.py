#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
import os
import cv2
import numpy as np
from datetime import datetime
import threading
from cv_bridge import CvBridge

class FrameRecorderNode(Node):
    def __init__(self):
        super().__init__('record_camera')

        # Declare parameters
        self.declare_parameter('topic', '/camera/image_raw/compressed')
        self.declare_parameter('output_directory', '')
        self.declare_parameter('fps', 10)
        self.declare_parameter('format', 'jpg')
        
        # Get parameters
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_directory').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.format = self.get_parameter('format').get_parameter_value().string_value
        
        # Create output directory structure
        if not self.output_dir:
            self.output_dir = os.getcwd()
        
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.topic_name = self.topic.replace('/', '_').strip('_')
        self.save_dir = os.path.join(self.output_dir, self.timestamp, self.topic_name)
        
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        self.get_logger().info(f"Will save frames to: {self.save_dir}")
        
        # Initialize bridge and subscription
        self.bridge = CvBridge()
        self.subscription = None
        
        # Recording state
        self.is_recording = False
        self.lock = threading.Lock()
        self.frame_count = 0
        
        # Create services
        self.start_srv = self.create_service(
            Trigger, 
            f'{self.get_name()}/start_recording', 
            self.start_recording_callback
        )
        
        self.stop_srv = self.create_service(
            Trigger, 
            f'{self.get_name()}/stop_recording', 
            self.stop_recording_callback
        )
        
        # Status timer
        self.timer = self.create_timer(5.0, self.report_status)
        
        self.get_logger().info(f"Frame recorder initialized for topic: {self.topic}")
        self.get_logger().info(f"Use services '{self.get_name()}/start_recording' and '{self.get_name()}/stop_recording' to control recording")

    def start_subscription(self):
        """Create the subscription to the image topic"""
        if self.subscription is None:
            self.subscription = self.create_subscription(
                CompressedImage,
                self.topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f"Subscribed to: {self.topic}")

    def stop_subscription(self):
        """Destroy the subscription if it exists"""
        if self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.get_logger().info(f"Unsubscribed from: {self.topic}")

    def image_callback(self, msg):
        """Process incoming compressed image messages"""
        if not self.is_recording:
            return
            
        with self.lock:
            try:
                # Convert compressed image to OpenCV format
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # Save the frame
                frame_filename = f"frame_{self.frame_count:06d}.{self.format}"
                save_path = os.path.join(self.save_dir, frame_filename)
                
                cv2.imwrite(save_path, cv_image)
                
                self.frame_count += 1
                
            except Exception as e:
                self.get_logger().error(f"Error processing frame: {e}")

    def start_recording_callback(self, request, response):
        """Service callback to start recording"""
        with self.lock:
            if self.is_recording:
                response.success = False
                response.message = "Already recording"
            else:
                self.is_recording = True
                self.start_subscription()
                response.success = True
                response.message = f"Started recording from {self.topic} to {self.save_dir}"
                self.get_logger().info(response.message)
        return response

    def stop_recording_callback(self, request, response):
        """Service callback to stop recording"""
        with self.lock:
            if not self.is_recording:
                response.success = False
                response.message = "Not recording"
            else:
                self.is_recording = False
                self.stop_subscription()
                response.success = True
                response.message = f"Stopped recording. Saved {self.frame_count} frames to {self.save_dir}"
                self.get_logger().info(response.message)
        return response

    def report_status(self):
        """Periodically report status"""
        if self.is_recording:
            self.get_logger().info(f"Recording active. Frames captured: {self.frame_count}")

    def destroy_node(self):
        """Clean up when node is shutdown"""
        with self.lock:
            if self.is_recording:
                self.is_recording = False
                self.stop_subscription()
                self.get_logger().info(f"Recording stopped on shutdown. Saved {self.frame_count} frames")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FrameRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Frame recorder error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()