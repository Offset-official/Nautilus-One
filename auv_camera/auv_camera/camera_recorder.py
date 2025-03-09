#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_srvs.srv import Trigger
import os
from datetime import datetime
import numpy as np
import cv2
import argparse
from threading import Lock


class MultiCameraFrameRecorder(Node):
    def __init__(self, camera_names):
        """Initialize the multi-camera frame recorder node.
        
        Args:
            camera_names (list): List of camera names to subscribe to (e.g., ['auv_camera_down', 'auv_camera_front'])
        """
        super().__init__('multi_camera_frame_recorder')
        
        self.camera_names = camera_names
        self.get_logger().info(f"Initializing recorder for cameras: {', '.join(camera_names)}")
        
        # Recording state
        self.is_recording = False
        self.frame_counts = {camera: 0 for camera in camera_names}
        self.recording_dir = ""
        self.lock = Lock()
        
        # Store the latest camera info for each camera
        self.latest_camera_infos = {camera: None for camera in camera_names}
        
        # Create subscribers and directories for each camera
        self.compressed_subs = {}
        self.camera_info_subs = {}
        self.topic_dirs = {}
        
        for camera in camera_names:
            compressed_topic = f"{camera}/image_raw/compressed"
            camera_info_topic = f"{camera}/camera_info"
            
            # Create subscribers
            self.compressed_subs[camera] = self.create_subscription(
                CompressedImage,
                compressed_topic,
                lambda msg, c=camera: self.compressed_image_callback(msg, c),
                10
            )
            
            self.camera_info_subs[camera] = self.create_subscription(
                CameraInfo,
                camera_info_topic,
                lambda msg, c=camera: self.camera_info_callback(msg, c),
                10
            )
            
            self.get_logger().info(f"Subscribed to {compressed_topic}")
            self.get_logger().info(f"Subscribed to {camera_info_topic}")
        
        # Create global services for starting and stopping recording
        self.start_recording_srv = self.create_service(
            Trigger,
            'start_recording',
            self.start_recording_callback
        )
        
        self.stop_recording_srv = self.create_service(
            Trigger,
            'stop_recording',
            self.stop_recording_callback
        )
        
        self.get_logger().info("Created global recording services")

    def camera_info_callback(self, msg, camera_name):
        """Callback for camera info messages."""
        self.latest_camera_infos[camera_name] = msg
        
    def compressed_image_callback(self, msg, camera_name):
        """Callback for compressed image messages."""
        with self.lock:
            if not self.is_recording:
                return
                
            try:
                # Convert compressed image to OpenCV format
                np_arr = np.frombuffer(msg.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if image is None:
                    self.get_logger().error(f"Failed to decode image for {camera_name}")
                    return
                
                # Create a filename with timestamp and frame number
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                frame_count = self.frame_counts[camera_name]
                filename = f"frame_{frame_count:06d}_{timestamp:.6f}.png"
                filepath = os.path.join(self.topic_dirs[camera_name], filename)
                
                # Save the image
                cv2.imwrite(filepath, image)
                
                # Save camera info as yaml if available and it's the first frame
                if self.latest_camera_infos[camera_name] is not None and frame_count == 0:
                    self._save_camera_info(camera_name)
                
                self.frame_counts[camera_name] += 1
                
                if self.frame_counts[camera_name] % 10 == 0:
                    self.get_logger().info(f"Saved frame {self.frame_counts[camera_name]} for {camera_name}")
                    
            except Exception as e:
                self.get_logger().error(f"Error processing image for {camera_name}: {e}")

    def _save_camera_info(self, camera_name):
        """Save camera info as a YAML file."""
        info = self.latest_camera_infos[camera_name]
        if info is None:
            return
            
        info_path = os.path.join(self.topic_dirs[camera_name], "camera_info.yaml")
        
        with open(info_path, 'w') as f:
            f.write(f"# Camera info for {camera_name}\n")
            f.write(f"frame_id: {info.header.frame_id}\n")
            f.write(f"height: {info.height}\n")
            f.write(f"width: {info.width}\n")
            f.write(f"distortion_model: {info.distortion_model}\n")
            f.write(f"D: {list(info.d)}\n")
            f.write(f"K: {list(info.k)}\n")
            f.write(f"R: {list(info.r)}\n")
            f.write(f"P: {list(info.p)}\n")
            
        self.get_logger().info(f"Saved camera info for {camera_name}")

    def start_recording_callback(self, request, response):
        """Service callback to start recording frames."""
        with self.lock:
            if self.is_recording:
                response.success = False
                response.message = "Already recording"
                return response
                
            # Create a directory structure for this recording session
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.recording_dir = os.path.join(os.getcwd(), f"camera_recordings_{timestamp}")
            
            # Create subdirectories for each camera
            self.topic_dirs = {}
            for camera in self.camera_names:
                self.topic_dirs[camera] = os.path.join(self.recording_dir, camera)
                os.makedirs(self.topic_dirs[camera], exist_ok=True)
                self.frame_counts[camera] = 0
            
            self.is_recording = True
            
            self.get_logger().info(f"Started recording to {self.recording_dir}")
            
            response.success = True
            response.message = f"Started recording from all cameras to {self.recording_dir}"
            return response

    def stop_recording_callback(self, request, response):
        """Service callback to stop recording frames."""
        with self.lock:
            if not self.is_recording:
                response.success = False
                response.message = "Not currently recording"
                return response
                
            self.is_recording = False
            
            # Create summary of frames recorded
            frame_summary = ", ".join([f"{camera}: {count} frames" 
                                       for camera, count in self.frame_counts.items()])
            
            self.get_logger().info(f"Stopped recording. {frame_summary}")
            
            response.success = True
            response.message = f"Stopped recording. {frame_summary}"
            return response


def main():
    parser = argparse.ArgumentParser(
        description='ROS2 node to record frames from multiple camera topics'
    )
    parser.add_argument(
        'camera_names',
        nargs='+',
        type=str,
        help='List of camera names (e.g., auv_camera_down auv_camera_front)'
    )
    
    # Parse arguments before initializing ROS 
    args = parser.parse_args()
    
    rclpy.init()
    node = MultiCameraFrameRecorder(args.camera_names)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()