#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class RepublishSdpNode : public rclcpp::Node
{
public:
  RepublishSdpNode(const std::string & sdp_path,
                   const std::string & image_topic,
                   const std::string & camera_info_topic)
  : Node("republish_sdp_low_latency_node")
  {
    // GStreamer pipeline configured for minimal latency:
    // 1) Read the SDP file (filesrc location=).
    // 2) Parse with sdpdemux.
    // 3) rtpjitterbuffer latency=0 => no extra buffer (lowest latency, higher risk of jitter).
    // 4) Depay (rtph264depay), parse (h264parse), decode (avdec_h264).
    // 5) Convert to raw frames (videoconvert).
    // 6) appsink sync=false => push frames out ASAP, ignoring playback clock.
    //
    // This is the typical minimal-latency pipeline. Adjust if you see excessive jitter or dropped frames.
    pipeline_ =
      "filesrc location=" + sdp_path + " ! "
      "sdpdemux ! "
      "rtpjitterbuffer latency=0 ! "  // minimal latency
      "rtph264depay ! "
      "h264parse ! "
      "avdec_h264 ! "
      "videoconvert ! "
      "appsink sync=false";           // ignore clock, push frames immediately

    RCLCPP_INFO(this->get_logger(), "Using low-latency GStreamer pipeline:\n%s", pipeline_.c_str());

    // Open the pipeline with CAP_GSTREAMER
    cap_.open(pipeline_, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open SDP stream from '%s'.", sdp_path.c_str());
      rclcpp::shutdown();
      return;
    }

    // Publishers: one for Image, one for CameraInfo
    pub_image_       = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
    pub_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Publishing camera frames on topic '%s'.", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing camera info on topic '%s'.", camera_info_topic.c_str());

    // Timer to grab frames ~30 Hz (adjust as needed for your camera's actual frame rate)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&RepublishSdpNode::timerCallback, this)
    );
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), 
                  "Empty frame received. The stream may have ended or is disconnected.");
      return;
    }

    // Convert to ROS Image
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    // Stamp & frame_id
    image_msg->header.stamp    = this->now();
    image_msg->header.frame_id = "camera_frame";

    // Publish the image
    pub_image_->publish(*image_msg);

    // Publish a default CameraInfo
    sensor_msgs::msg::CameraInfo cam_info;
    cam_info.header = image_msg->header;  // same timestamp/frame
    cam_info.width  = static_cast<uint32_t>(frame.cols);
    cam_info.height = static_cast<uint32_t>(frame.rows);

    cam_info.distortion_model = "plumb_bob";
    cam_info.d.resize(5, 0.0);

    // Example intrinsics
    double fx = 500.0, fy = 500.0;
    double cx = frame.cols / 2.0, cy = frame.rows / 2.0;

    cam_info.k = {
      fx,   0.0, cx,
      0.0,  fy,  cy,
      0.0,  0.0, 1.0
    };
    cam_info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    cam_info.p = {
      fx,   0.0, cx,  0.0,
      0.0,  fy,  cy,  0.0,
      0.0,  0.0, 1.0, 0.0
    };

    pub_camera_info_->publish(cam_info);
  }

  std::string pipeline_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 4) {
    std::cerr << "Usage: ros2 run <your_package> republish_sdp_low_latency "
                 "<path_to_sdp_file> <image_topic> <camera_info_topic>\n";
    return 1;
  }

  std::string sdp_path          = argv[1];
  std::string image_topic       = argv[2];
  std::string camera_info_topic = argv[3];

  auto node = std::make_shared<RepublishSdpNode>(sdp_path, image_topic, camera_info_topic);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
