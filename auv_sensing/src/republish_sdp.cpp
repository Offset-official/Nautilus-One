#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class RepublishSdpNode : public rclcpp::Node
{
public:
  RepublishSdpNode(const std::string & sdp_path, const std::string & image_topic)
  : Node("republish_sdp_node")
  {
    // Build a GStreamer pipeline that:
    //   1. Reads the local SDP file (filesrc location=...).
    //   2. Uses sdpdemux to parse it.
    //   3. Minimizes latency with rtpjitterbuffer latency=0 (you can adjust).
    //   4. Depayloads, parses, decodes H.264.
    //   5. Converts to raw frames for appsink, which feeds OpenCV.
    pipeline_ = "filesrc location=" + sdp_path +
                " ! "
                "sdpdemux ! "
                "rtpjitterbuffer latency=0 ! "
                "rtph264depay ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "appsink sync=false";

    RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline:\n%s", pipeline_.c_str());

    // Open the pipeline in OpenCV with CAP_GSTREAMER
    cap_.open(pipeline_, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open SDP stream from '%s'.", sdp_path.c_str());
      rclcpp::shutdown();
      return;
    }

    // Create the publisher for sensor_msgs/Image
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
    RCLCPP_INFO(this->get_logger(), "Publishing camera frames on topic '%s'.", image_topic.c_str());

    // Timer callback at ~30fps (adjust as needed)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),  // ~30 Hz
      std::bind(&RepublishSdpNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(
        this->get_logger(), "Empty frame received. Check if the stream ended or is disconnected.");
      return;
    }

    // Convert the frame to a ROS Image message
    auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    image_msg->header.stamp = this->now();

    // Publish the message
    pub_->publish(*image_msg);
  }

  std::string pipeline_;
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: ros2 run <your_package> republish_sdp <path_to_sdp_file> <output_topic>\n";
    return 1;
  }

  std::string sdp_path = argv[1];
  std::string image_topic = argv[2];

  auto node = std::make_shared<RepublishSdpNode>(sdp_path, image_topic);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
