#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class CameraPublisherNode : public rclcpp::Node
{
public:
  CameraPublisherNode(const std::string & image_topic, const std::string & camera_info_topic)
  : Node("webcam_publisher")
  {
    // Create publishers
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);

    // Create a timer to periodically publish messages (30Hz)
    timer_ = this->create_wall_timer(
      33ms,  // ~30Hz
      std::bind(&CameraPublisherNode::publishData, this)
    );

    RCLCPP_INFO(this->get_logger(), 
                "Publishing camera data on image topic: '%s' and camera info topic: '%s'",
                image_topic.c_str(), camera_info_topic.c_str());
  }

private:
  void publishData()
  {
    // Create and populate a dummy image
    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = this->now();
    image_msg.header.frame_id = "camera_frame";
    image_msg.height = 480;
    image_msg.width = 640;
    image_msg.encoding = "rgb8";
    image_msg.step = image_msg.width * 3; // 3 bytes per pixel in RGB8
    image_msg.data.resize(image_msg.height * image_msg.step, 255); // fill with white

    // Create and populate corresponding CameraInfo
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header = image_msg.header;
    camera_info_msg.height = image_msg.height;
    camera_info_msg.width = image_msg.width;

    // For a real camera, these values should be set properly.
    // e.g., camera_info_msg.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    //       camera_info_msg.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};

    // Publish both
    image_publisher_->publish(image_msg);
    camera_info_publisher_->publish(camera_info_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Usage: publish_webcam <image_topic> <camera_info_topic>\n"
                 "Example: publish_webcam /camera/image_raw /camera/camera_info");
    return 1;
  }
  std::string image_topic = argv[1];
  std::string camera_info_topic = argv[2];

  auto node = std::make_shared<CameraPublisherNode>(image_topic, camera_info_topic);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}