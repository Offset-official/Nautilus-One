#include <opencv2/opencv.hpp>

#include "auv_interfaces/msg/detection_array.hpp"
#include "auv_interfaces/srv/yolo_inference.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class CameraInferenceNode : public rclcpp::Node
{
public:
  CameraInferenceNode()
  : Node("infer_camera_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting node...");

    auto topics = this->get_topic_names_and_types();
    bool topic_exists = topics.find(camera_source_topic_) != topics.end();
    if (topic_exists) {
      RCLCPP_INFO(this->get_logger(), "Topic %s is alive", camera_source_topic_);
      RCLCPP_INFO(this->get_logger(), "Start Gazebo to start inference.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Topic %s is not alive. Exiting...", camera_source_topic_);
      rclcpp::shutdown();
    }

    try {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_source_topic_, 1,
        std::bind(&CameraInferenceNode::image_callback, this, std::placeholders::_1));
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(camera_pub_topic_, 10);
      client_ = this->create_client<auv_interfaces::srv::YoloInference>(inference_service_);
      detectionsPublisher_ =
        this->create_publisher<auv_interfaces::msg::DetectionArray>(detections_pub_topic_, 10);

    } catch (rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
      rclcpp::shutdown();
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<auv_interfaces::msg::DetectionArray>::SharedPtr detectionsPublisher_;
  const char * camera_source_topic_ = "/auv_camera/image_raw";
  const char * camera_pub_topic_ = "/auv_camera/image_inferred";
  const char * detections_pub_topic_ = "/auv_camera/detections";
  const char * inference_service_ = "yolo_inference_server";
  rclcpp::Client<auv_interfaces::srv::YoloInference>::SharedPtr client_;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    try {
      auto request = std::make_shared<auv_interfaces::srv::YoloInference::Request>();
      request->image = *msg;

      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }

      auto future = client_->async_send_request(
        request, [this](rclcpp::Client<auv_interfaces::srv::YoloInference>::SharedFuture response) {
          // Callback for handling the response
          if (response.valid()) {
            RCLCPP_INFO(this->get_logger(), "Got inference, publishing to %s", camera_pub_topic_);

            // auto publishImg = sensor_msgs::msg::Image();
            auto publishImg = response.get()->result_image;
            auto detections = response.get()->detections;
            auv_interfaces::msg::DetectionArray detectionArrayMsg;
            detectionArrayMsg.detections = detections;

            publisher_->publish(publishImg);
            detectionsPublisher_->publish(detectionArrayMsg);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", inference_service_);
          }
        });

    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Some exception happenend.");
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInferenceNode>());
  rclcpp::shutdown();
  return 0;
}
