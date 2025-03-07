#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import argparse
import os
import signal
import sys
from threading import Thread

class RecorderLauncher(Node):
    def __init__(self):
        super().__init__('recorder_launcher')
        
        # Get topics from arguments or use defaults
        self.declare_parameter('topics', [
            '/auv_camera_down/image_raw/compressed',
            '/auv_camera_front/image_raw/compressed',
            '/usb_cam_2/image_raw/compressed'
        ])
        
        self.topics = self.get_parameter('topics').get_parameter_value().string_array_value
        self.get_logger().info(f"Launching recorder nodes for topics: {self.topics}")
        
        # Dictionary to keep track of running processes
        self.processes = {}
        
        # Start recorder nodes
        for topic in self.topics:
            self.start_recorder(topic)
            
        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def start_recorder(self, topic):
        # Replace slashes with underscores for node name
        topic_clean = topic.replace('/', '_').lstrip('_')
        node_name = f"frame_recorder_{topic_clean}"
        
        # Construct command to launch recorder node
        cmd = [
            'ros2', 'run', 'auv_camera', 'frame_recorder',
            '--ros-args',
            '-p', f'topic:={topic}',
            '--remap', f'__node:={node_name}'
        ]
        
        self.get_logger().info(f"Starting recorder node: {' '.join(cmd)}")
        
        # Start the process
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Store process
        self.processes[topic] = process
        
        # Start threads to monitor output
        Thread(target=self.monitor_output, args=(process.stdout, f"{node_name} [OUT]"), daemon=True).start()
        Thread(target=self.monitor_output, args=(process.stderr, f"{node_name} [ERR]"), daemon=True).start()
    
    def monitor_output(self, pipe, prefix):
        try:
            for line in pipe:
                if line.strip():
                    self.get_logger().info(f"{prefix}: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error monitoring process output: {e}")
    
    def signal_handler(self, sig, frame):
        self.get_logger().info("Shutdown signal received, terminating recorder nodes...")
        self.shutdown()
        sys.exit(0)
    
    def shutdown(self):
        # Terminate all processes
        for topic, process in self.processes.items():
            if process.poll() is None:  # If still running
                self.get_logger().info(f"Terminating recorder for {topic}")
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.get_logger().warning(f"Process for {topic} did not terminate, forcing kill")
                    process.kill()

def main():
    rclpy.init()
    node = RecorderLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  