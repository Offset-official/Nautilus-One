#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class ShowCameraSdpNode : public rclcpp::Node
{
public:
  explicit ShowCameraSdpNode(const std::string & sdp_path) : Node("show_camera_sdp")
  {
    // GStreamer pipeline reading from an SDP file via filesrc + sdpdemux
    // Then it buffers, depayloads, parses, decodes H.264, converts color, and hands frames to appsink.
    //
    // The "rtpjitterbuffer latency=0" attempts to minimize latency; you can increase it if you see jitter/dropped frames.
    // The 'queue' elements can help ensure smooth pipeline operation. Adjust as needed.
    //
    // NOTE: The pipeline references only video. If your SDP has audio also, sdpdemux might create multiple pads.
    //       This pipeline focuses on the video pad.

    pipeline_ = "filesrc location=" + sdp_path +
                " ! "
                "sdpdemux ! "
                "rtpjitterbuffer latency=0 ! "
                "rtph264depay ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "queue ! "
                "appsink sync=false";

    RCLCPP_INFO(this->get_logger(), "Using GStreamer pipeline:\n%s", pipeline_.c_str());

    // Open the pipeline with OpenCV
    cap_.open(pipeline_, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open SDP stream with GStreamer pipeline");
      rclcpp::shutdown();
      return;
    }

    // Create a timer to periodically grab frames
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),  // ~33 fps
      std::bind(&ShowCameraSdpNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN(
        this->get_logger(), "Empty frame received; stream may have ended or become unavailable.");
      return;
    }
    // Show frame in a window
    cv::imshow("SDP Camera (GStreamer, Low Latency)", frame);
    cv::waitKey(1);
  }

  std::string pipeline_;
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run <package_name> show_camera_sdp <path_to_sdp>\n";
    return 1;
  }

  std::string sdp_path = argv[1];
  auto node = std::make_shared<ShowCameraSdpNode>(sdp_path);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
