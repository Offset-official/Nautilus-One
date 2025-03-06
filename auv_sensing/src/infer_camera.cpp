#include <opencv2/opencv.hpp>
#include "auv_interfaces/msg/detection_array.hpp"
#include "auv_interfaces/srv/yolo_inference.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"  // For compressed images

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
    // New parameter indicating whether the source image is compressed
    this->declare_parameter<bool>("compressed", false);

    // Get parameter values
    camera_source_topic_  = this->get_parameter("camera_source_topic").as_string();
    camera_pub_topic_     = this->get_parameter("camera_pub_topic").as_string();
    detections_pub_topic_ = this->get_parameter("detections_pub_topic").as_string();
    inference_service_    = this->get_parameter("inference_service").as_string();
    is_compressed_        = this->get_parameter("compressed").as_bool();

    RCLCPP_INFO(get_logger(), "Starting node...");
    RCLCPP_INFO(get_logger(), "Subscribing to topic: %s", camera_source_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Compressed parameter: %s", is_compressed_ ? "true" : "false");

    // (Optional) Check if topic exists...
    // RCLCPP_INFO(get_logger(), "Topic '%s' is alive", camera_source_topic_.c_str());
    // RCLCPP_INFO(get_logger(), "Start Gazebo to start inference.");

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
  bool is_compressed_;

  rclcpp::SubscriptionBase::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<auv_interfaces::msg::DetectionArray>::SharedPtr detectionsPublisher_;
  rclcpp::Client<auv_interfaces::srv::YoloInference>::SharedPtr client_;

  // Setup the image subscription based on whether the source is compressed.
  void setupSubscriptions()
  {
    if (is_compressed_) {
      subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        camera_source_topic_,
        1,
        std::bind(&CameraInferenceNode::compressedImageCallback, this, std::placeholders::_1)
      );
    } else {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_source_topic_,
        1,
        std::bind(&CameraInferenceNode::imageCallback, this, std::placeholders::_1)
      );
    }
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

  // Callback for uncompressed image messages
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    processImage(msg);
  }

  // Callback for compressed image messages
  void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try {
      // Wrap the compressed image data in a cv::Mat
      cv::Mat encoded_img(1, msg->data.size(), CV_8UC1,
                          const_cast<unsigned char*>(msg->data.data()));
      // Decode the image (assuming MJPEG compression)
      cv::Mat decoded_img = cv::imdecode(encoded_img, cv::IMREAD_COLOR);
      if (decoded_img.empty()) {
        RCLCPP_ERROR(get_logger(), "Failed to decode compressed image.");
        return;
      }
      // Convert the decoded image to a sensor_msgs::msg::Image using cv_bridge
      sensor_msgs::msg::Image::SharedPtr image_msg;
      try {
        // image_msg = cv_bridge::CvImage(msg->header, "bgr8", decoded_img)->toImageMsg();
        cv_bridge::CvImage cv_img(msg->header, "bgr8", decoded_img);
sensor_msgs::msg::Image::SharedPtr image_msg = cv_img.toImageMsg();

      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      processImage(image_msg);
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception in compressedImageCallback: %s", e.what());
    }
  }

  // Process the image and call the inference service.
  void processImage(const sensor_msgs::msg::Image::SharedPtr & img_msg)
  {
    auto request = std::make_shared<auv_interfaces::srv::YoloInference::Request>();
    request->image = *img_msg;

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
