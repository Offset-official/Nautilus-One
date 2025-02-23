#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "auv_interfaces/srv/yolo_inference.hpp"
#include "auv_interfaces/msg/detection_array.hpp"

using namespace std::chrono_literals;

class CameraInferenceNode : public rclcpp::Node 
{
public:
  CameraInferenceNode() : Node("infer_camera_node")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("camera_source_topic", "/auv_camera/image_raw");
    this->declare_parameter<std::string>("camera_pub_topic", "/auv_camera/image_inferred");
    this->declare_parameter<std::string>("detections_pub_topic", "/auv_camera/detections");
    this->declare_parameter<std::string>("inference_service", "yolo_inference_server");

    // Get parameter values
    camera_source_topic_   = this->get_parameter("camera_source_topic").as_string();
    camera_pub_topic_      = this->get_parameter("camera_pub_topic").as_string();
    detections_pub_topic_  = this->get_parameter("detections_pub_topic").as_string();
    inference_service_     = this->get_parameter("inference_service").as_string();

    RCLCPP_INFO(get_logger(), "Starting node...");

    if (!checkTopicExists(camera_source_topic_)) {
      RCLCPP_ERROR(get_logger(), "Topic '%s' is not alive. Exiting...", camera_source_topic_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Topic '%s' is alive", camera_source_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Start Gazebo to start inference.");

    setupSubscriptions();
    setupPublishers();
    setupClient();
  }

private:
  // Parameters as member variables
  std::string camera_source_topic_;
  std::string camera_pub_topic_;
  std::string detections_pub_topic_;
  std::string inference_service_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<auv_interfaces::msg::DetectionArray>::SharedPtr detectionsPublisher_;
  rclcpp::Client<auv_interfaces::srv::YoloInference>::SharedPtr client_;

  // Check if a topic exists
  bool checkTopicExists(const std::string & topic)
  {
    auto topics = this->get_topic_names_and_types();
    return topics.find(topic) != topics.end();
  }

  // Setup the image subscription
  void setupSubscriptions()
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_source_topic_,
      1,
      std::bind(&CameraInferenceNode::imageCallback, this, std::placeholders::_1)
    );
  }

  // Setup publishers for the inferred image and detection array
  void setupPublishers()
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(camera_pub_topic_, 10);
    detectionsPublisher_ = this->create_publisher<auv_interfaces::msg::DetectionArray>(detections_pub_topic_, 10);
  }

  // Setup the inference service client
  void setupClient()
  {
    client_ = this->create_client<auv_interfaces::srv::YoloInference>(inference_service_);
  }

  // Wait for the inference service to become available
  bool waitForServiceAvailability()
  {
    while (!client_->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(get_logger(), "Service '%s' not available, waiting again...", inference_service_.c_str());
    }
    return true;
  }

  // Handle the asynchronous response from the inference service
  void handleServiceResponse(const rclcpp::Client<auv_interfaces::srv::YoloInference>::SharedFuture response)
  {
    if (response.valid())
    {
      RCLCPP_INFO(get_logger(), "Got inference, publishing to '%s'", camera_pub_topic_.c_str());
      auto result = response.get();

      // Publish the inferred image
      publisher_->publish(result->result_image);

      // Publish the detection array message
      auv_interfaces::msg::DetectionArray detectionArrayMsg;
      detectionArrayMsg.detections = result->detections;
      detectionsPublisher_->publish(detectionArrayMsg);
    }
    else {
      RCLCPP_ERROR(get_logger(), "Failed to call service '%s'", inference_service_.c_str());
    }
  }

  // Callback triggered when a new image is received from the camera topic
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto request = std::make_shared<auv_interfaces::srv::YoloInference::Request>();
    request->image = *msg;

    if (!waitForServiceAvailability()) {
      rclcpp::shutdown();
      return;
    }

    auto future = client_->async_send_request(
      request,
      std::bind(&CameraInferenceNode::handleServiceResponse, this, std::placeholders::_1)
    );
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraInferenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
