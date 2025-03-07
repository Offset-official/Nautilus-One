#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import gi
import threading
import time
import numpy as np
import cv2

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class MJPEG720pCompressedPublisher(Node):
    def __init__(self):
        super().__init__('mjpeg_720p_compressed_publisher')

        # Parameters: device path, camera name, and frame rate.
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('camera_name', 'mjpeg_720p_cam')
        self.declare_parameter('frame_rate', 10)

        camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value

        self.get_logger().info(f"Camera device: {camera_device}")
        self.get_logger().info(f"Camera name: {camera_name}")
        self.get_logger().info(f"Frame rate: {frame_rate}")

        # Topics following image_transport conventions.
        # Publish the compressed image on the topic "image_raw/compressed".
        image_topic = f'{camera_name}/image_raw/compressed'
        info_topic = f'{camera_name}/camera_info'
        self.get_logger().info(f"Publishing sensor_msgs/CompressedImage on: {image_topic}")
        self.get_logger().info(f"Publishing CameraInfo on: {info_topic}")

        # Publishers for compressed image and camera info.
        self.image_pub = self.create_publisher(CompressedImage, image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, info_topic, 10)

        # Initialize GStreamer with a pipeline that decodes MJPEG into a raw BGR image.
        # Pipeline breakdown:
        #   - v4l2src: grabs images from the camera.
        #   - image/jpeg: enforces MJPEG at 1280x720 with the specified framerate.
        #   - jpegdec: decodes the MJPEG into raw image data.
        #   - videoconvert: converts the image to a standard format.
        #   - video/x-raw,format=BGR: outputs a raw BGR image.
        #   - appsink: provides access to the frames in our Python code.
        Gst.init(None)
        pipeline_str = (
            f'v4l2src device={camera_device} ! '
            f'image/jpeg,width=1280,height=720,framerate={frame_rate}/1 ! '
            'jpegdec ! videoconvert ! video/x-raw,format=BGR ! '
            'appsink name=appsink emit-signals=true max-buffers=1 drop=true'
        )
        self.get_logger().info(f"GStreamer pipeline: {pipeline_str}")

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.connect('new-sample', self.on_new_sample)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set pipeline to PLAYING")
            raise RuntimeError("Could not start GStreamer pipeline")

        # Start the GLib main loop in a separate thread for GStreamer callbacks.
        self.loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.glib_thread.start()
        self.get_logger().info("GStreamer pipeline started...")

        # For diagnostics: track frame count and report FPS.
        self.frame_count = 0
        self.last_time = time.time()
        self.timer = self.create_timer(5.0, self.report_status)

        # Consistent frame ID for image and camera info messages.
        self.frame_id = f"{camera_name}_frame"

    def on_new_sample(self, sink):
        """Called by GStreamer when a new raw frame is available."""
        try:
            sample = sink.emit('pull-sample')
            if sample is None:
                return Gst.FlowReturn.OK

            buf = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')

            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                self.get_logger().error("Failed to map GStreamer buffer.")
                return Gst.FlowReturn.ERROR

            # Convert the raw buffer data to a NumPy array.
            data = np.frombuffer(map_info.data, np.uint8)
            try:
                # Reshape assuming a BGR image (3 channels).
                frame = data.reshape((height, width, 3))
            except Exception as e:
                self.get_logger().error(f"Reshape error: {e}")
                buf.unmap(map_info)
                return Gst.FlowReturn.ERROR

            buf.unmap(map_info)

            # Compress the raw BGR image to JPEG.
            ret, jpeg = cv2.imencode('.jpg', frame)
            if not ret:
                self.get_logger().error("Failed to compress image to JPEG.")
                return Gst.FlowReturn.ERROR

            # Construct the CompressedImage message.
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.header.frame_id = self.frame_id
            compressed_msg.format = "jpeg"  # Indicate the compression format.
            compressed_msg.data = jpeg.tobytes()

            # Publish the compressed image.
            self.image_pub.publish(compressed_msg)

            # Publish a corresponding CameraInfo message.
            cam_info = CameraInfo()
            cam_info.header.stamp = compressed_msg.header.stamp
            cam_info.header.frame_id = self.frame_id
            cam_info.width = width
            cam_info.height = height
            cam_info.distortion_model = 'plumb_bob'
            cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            cam_info.k = [
                1.0, 0.0, width / 2.0,
                0.0, 1.0, height / 2.0,
                0.0, 0.0, 1.0
            ]
            cam_info.r = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            ]
            cam_info.p = [
                1.0, 0.0, width / 2.0, 0.0,
                0.0, 1.0, height / 2.0, 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            self.info_pub.publish(cam_info)

            self.frame_count += 1
            return Gst.FlowReturn.OK

        except Exception as e:
            self.get_logger().error(f"Error in on_new_sample: {e}")
            return Gst.FlowReturn.ERROR

    def report_status(self):
        """Periodically log the pipeline state and approximate FPS."""
        ret, state, _pending = self.pipeline.get_state(0)
        now = time.time()
        elapsed = now - self.last_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f"Pipeline state: {state.value_nick}, frames: {self.frame_count}, FPS: {fps:.1f}"
        )
        self.frame_count = 0
        self.last_time = now

    def destroy_node(self):
        self.get_logger().info("Shutting down pipeline...")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
        if self.glib_thread.is_alive():
            self.glib_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MJPEG720pCompressedPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Camera node error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
