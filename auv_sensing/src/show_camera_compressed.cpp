#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

/**
 * @brief A simple node that subscribes to a CompressedImage topic and displays the images using OpenCV.
 *
 */
class ShowCameraCompressedNode : public rclcpp::Node
{
public:
  explicit ShowCameraCompressedNode(const char * camera_topic) : Node("show_camera_compressed")
  {
    RCLCPP_INFO(this->get_logger(), "Starting node...");

    // Subscribe to a sensor_msgs::msg::CompressedImage topic
    try {
      subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        camera_topic, 10,
        std::bind(&ShowCameraCompressedNode::imageCallback, this, std::placeholders::_1));
    } catch (rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
      rclcpp::shutdown();
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) const
  {
    try {
      // The compressed image data is in msg->data, which is a std::vector<uint8_t>.
      // We can decode it using cv::imdecode.

      // Convert std::vector<uint8_t> to a cv::Mat that can be used by cv::imdecode.
      // We wrap the raw data in a 1D Mat of type CV_8UC1 (8-bit, single-channel).
      cv::Mat compressed_data(
        1, static_cast<int>(msg->data.size()), CV_8UC1, (void *)msg->data.data());

      // Decode the compressed image to a color (BGR) Mat
      cv::Mat frame = cv::imdecode(compressed_data, cv::IMREAD_COLOR);

      if (frame.empty()) {
        RCLCPP_WARN(
          this->get_logger(), "Decoded image is empty. Check if the compressed data is valid.");
        return;
      }

      cv::imshow("Camera (Compressed)", frame);
      // Use a small (non-zero) waitKey so the GUI thread can process events:
      cv::waitKey(1);
    } catch (const cv::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception during decompression: %s", e.what());
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: show_camera_compressed <compressed_image_topic>\n";
    return 1;
  }

  const char * camera_topic = argv[1];

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShowCameraCompressedNode>(camera_topic));
  rclcpp::shutdown();

  return 0;
}
