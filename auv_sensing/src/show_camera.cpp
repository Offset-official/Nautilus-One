#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

class CompressedImageViewerNode : public rclcpp::Node
{
public:
  CompressedImageViewerNode(const std::string & topic_name)
  : Node("compressed_image_viewer")
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing to compressed topic: %s", topic_name.c_str());
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_name, 10,
      std::bind(&CompressedImageViewerNode::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try {
      // Wrap the compressed image data in a cv::Mat
      cv::Mat encoded_img(1, msg->data.size(), CV_8UC1,
                          const_cast<unsigned char*>(msg->data.data()));
      
      // Decode the MJPEG image into a BGR (OpenCV) format
      cv::Mat decoded_img = cv::imdecode(encoded_img, cv::IMREAD_COLOR);
      if (decoded_img.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image.");
        return;
      }
      
      // Display the decoded image in a window
      cv::imshow("Compressed Image Viewer", decoded_img);
      cv::waitKey(1);
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <compressed_image_topic>\n";
    std::cerr << "Example: " << argv[0] << " /auv_front_camera/image_raw/compressed\n";
    return 1;
  }
  
  std::string topic_name = argv[1];
  auto node = std::make_shared<CompressedImageViewerNode>(topic_name);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  cv::destroyAllWindows();
  
  return 0;
}
