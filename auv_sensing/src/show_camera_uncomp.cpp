#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

class ImageViewerNode : public rclcpp::Node
{
public:
  ImageViewerNode(const std::string & topic_name)
  : Node("uncompressed_image_viewer")
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing to uncompressed topic: %s", topic_name.c_str());
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 10,
      std::bind(&ImageViewerNode::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert the ROS image message to an OpenCV Mat using cv_bridge
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

      // Display the image in a window
      cv::imshow("Image Viewer", cv_ptr->image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <image_topic>\n";
    std::cerr << "Example: " << argv[0] << " /auv_front_camera/image_raw\n";
    return 1;
  }
  
  std::string topic_name = argv[1];
  auto node = std::make_shared<ImageViewerNode>(topic_name);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  cv::destroyAllWindows();
  
  return 0;
}
