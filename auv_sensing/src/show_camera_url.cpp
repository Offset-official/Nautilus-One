#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

class ShowCameraRtspNode : public rclcpp::Node
{
public:
  explicit ShowCameraRtspNode(const std::string & camera_url) : Node("show_camera_rtsp")
  {
    RCLCPP_INFO(this->get_logger(), "Starting node with RTSP URL: %s", camera_url.c_str());

    // Attempt to open the video capture from the provided RTSP URL
    // You can specify different backends depending on how OpenCV was built:
    // e.g., cap_.open(camera_url, cv::CAP_FFMPEG) or cap_.open(camera_url, cv::CAP_GSTREAMER)
    cap_.open(camera_url, cv::CAP_FFMPEG);

    // Check if the capture was successful
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream: %s", camera_url.c_str());
      rclcpp::shutdown();
      return;
    }

    // Create a timer to periodically read and display frames
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),  // ~33 FPS, adjust as needed
      std::bind(&ShowCameraRtspNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(
        this->get_logger(), "Received an empty frame from RTSP stream. Check the camera or URL.");
      return;
    }

    // Display the frame
    cv::imshow("Camera Feed (RTSP)", frame);
    // Important: small wait time so OpenCV can process GUI events
    cv::waitKey(1);
  }

  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr
      << "Usage: show_camera_rtsp <rtsp_url>\n"
      << "Example: show_camera_rtsp rtsp://user:pass@192.168.x.x:554/Streaming/Channels/1\n";
    return 1;
  }

  auto node = std::make_shared<ShowCameraRtspNode>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
