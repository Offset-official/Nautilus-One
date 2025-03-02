#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import gi
import threading
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class MJPEG720pNode(Node):
    def __init__(self):
        super().__init__('mjpeg_720p_publisher')

        # Parameters: device path and camera name
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('camera_name', 'mjpeg_720p_cam')

        camera_device = self.get_parameter('camera_device').get_parameter_value().string_value
        camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # Topics
        image_topic = f'{camera_name}/image_raw/compressed'
        info_topic = f'{camera_name}/camera_info'

        self.get_logger().info(f"Camera device: {camera_device}")
        self.get_logger().info(f"Camera name: {camera_name}")
        self.get_logger().info(f"Publishing CompressedImage on: {image_topic}")
        self.get_logger().info(f"Publishing CameraInfo on: {info_topic}")

        # Publishers
        self.image_pub = self.create_publisher(CompressedImage, image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, info_topic, 10)

        # Initialize GStreamer
        Gst.init(None)

        # Use the camera's native MJPEG at 1280×720, 30 fps (assuming it supports that)
        pipeline_str = (
            f'v4l2src device={camera_device} ! '
            'image/jpeg,width=1280,height=720,framerate=30/1 ! '
            'appsink name=appsink emit-signals=true max-buffers=1 drop=true'
        )
        self.get_logger().info(f"GStreamer pipeline: {pipeline_str}")

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.connect('new-sample', self.on_new_sample)

        # Start GStreamer pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Failed to set pipeline to PLAYING")
            raise RuntimeError("Could not start GStreamer pipeline")

        self.loop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.loop.run, daemon=True)
        self.glib_thread.start()

        self.get_logger().info("GStreamer MJPEG 1280x720 pipeline started...")

        # For diagnostics
        self.frame_count = 0
        self.last_time = time.time()
        self.timer = self.create_timer(5.0, self.report_status)

        # Consistent frame ID for camera_info
        self.frame_id = f"{camera_name}_frame"

    def on_new_sample(self, sink):
        """Called by GStreamer when a new MJPEG frame arrives."""
        try:
            sample = sink.emit('pull-sample')
            if sample is None:
                return Gst.FlowReturn.OK

            buf = sample.get_buffer()
            caps = sample.get_caps()

            # Extract width & height from caps (should be 1280x720)
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')

            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                self.get_logger().error("Failed to map GStreamer buffer.")
                return Gst.FlowReturn.ERROR

            # Construct CompressedImage message
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.frame_id
            img_msg.format = 'jpeg'
            img_msg.data = bytes(map_info.data)

            buf.unmap(map_info)

            # Publish the compressed image
            self.image_pub.publish(img_msg)

            # Publish a dummy CameraInfo
            cam_info = CameraInfo()
            cam_info.header.stamp = img_msg.header.stamp
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
            cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            cam_info.p = [
                1.0, 0.0, width / 2.0, 0.0,
                0.0, 1.0, height / 2.0, 0.0,
                0.0, 0.0, 1.0,          0.0
            ]
            self.info_pub.publish(cam_info)

            self.frame_count += 1
            return Gst.FlowReturn.OK

        except Exception as e:
            self.get_logger().error(f"Error in on_new_sample: {e}")
            return Gst.FlowReturn.ERROR

    def report_status(self):
        """Periodically logs pipeline status & approximate FPS."""
        ret, state, _pending = self.pipeline.get_state(0)
        now = time.time()
        elapsed = now - self.last_time
        fps = 0.0
        if elapsed > 0.0:
            fps = self.frame_count / elapsed
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
        node = MJPEG720pNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Camera node error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
