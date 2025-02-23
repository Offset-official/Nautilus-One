#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "auv_interfaces/srv/image_color_detect.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ColorDetector : public rclcpp::Node
{
public:
  ColorDetector() : Node("color_detector")
  {
    declare_parameter("min_positive", min_positive);
    get_parameter("min_positive", min_positive);

    declare_parameter("red_hsv_ranges", red_hsv_filter_ranges_);
    get_parameter("red_hsv_ranges", red_hsv_filter_ranges_);

    declare_parameter("green_hsv_ranges", green_hsv_filter_ranges_);
    get_parameter("green_hsv_ranges", green_hsv_filter_ranges_);

    declare_parameter("blue_hsv_ranges", blue_hsv_filter_ranges_);
    get_parameter("blue_hsv_ranges", blue_hsv_filter_ranges_);

    // Declare the service
    color_detection_service_ = this->create_service<auv_interfaces::srv::ImageColorDetect>(
      "detect_color", std::bind(&ColorDetector::detect_color_service_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Ready to detect color");
  }

private:
  rclcpp::Service<auv_interfaces::srv::ImageColorDetect>::SharedPtr color_detection_service_;

  // HSV ranges for detection [h,s,v H,S,V]
  std::vector<double> red_hsv_filter_ranges_{0, 0, 0, 180, 255, 255};
  std::vector<double> green_hsv_filter_ranges_{0, 0, 0, 180, 255, 255};
  std::vector<double> blue_hsv_filter_ranges_{0, 0, 0, 180, 255, 255};
  // number of pixels required to classify a correct color detection
  uint8_t min_positive = 200;

  // Service callback
  void detect_color_service_callback(
    const std::shared_ptr<auv_interfaces::srv::ImageColorDetect::Request> request,
    std::shared_ptr<auv_interfaces::srv::ImageColorDetect::Response> response)
  {
    const auto & msg = request->image;

    const float & red_h = red_hsv_filter_ranges_[0];
    const float & red_s = red_hsv_filter_ranges_[1];
    const float & red_v = red_hsv_filter_ranges_[2];
    const float & red_H = red_hsv_filter_ranges_[3];
    const float & red_S = red_hsv_filter_ranges_[4];
    const float & red_V = red_hsv_filter_ranges_[5];

    const float & green_h = green_hsv_filter_ranges_[0];
    const float & green_s = green_hsv_filter_ranges_[1];
    const float & green_v = green_hsv_filter_ranges_[2];
    const float & green_H = green_hsv_filter_ranges_[3];
    const float & green_S = green_hsv_filter_ranges_[4];
    const float & green_V = green_hsv_filter_ranges_[5];

    const float & blue_h = blue_hsv_filter_ranges_[0];
    const float & blue_s = blue_hsv_filter_ranges_[1];
    const float & blue_v = blue_hsv_filter_ranges_[2];
    const float & blue_H = blue_hsv_filter_ranges_[3];
    const float & blue_S = blue_hsv_filter_ranges_[4];
    const float & blue_V = blue_hsv_filter_ranges_[5];

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      response->color = "Error processing image";
      return;
    };

    cv::Mat img_hsv;
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

    cv::Mat1b filtered;
    std::string detected_color = "none";

    // RED color detection
    cv::inRange(
      img_hsv, cv::Scalar(red_h, red_s, red_v), cv::Scalar(red_H, red_S, red_V), filtered);
    auto numPositive = cv::countNonZero(filtered);
    if (numPositive >= min_positive) {
      detected_color = "red";
    }

    // GREEN color detection
    cv::inRange(
      img_hsv, cv::Scalar(green_h, green_s, green_v), cv::Scalar(green_H, green_S, green_V),
      filtered);
    numPositive = cv::countNonZero(filtered);
    if (numPositive >= min_positive) {
      detected_color = "green";
    }

    // BLUE color detection
    cv::inRange(
      img_hsv, cv::Scalar(blue_h, blue_s, blue_v), cv::Scalar(blue_H, blue_S, blue_V), filtered);
    numPositive = cv::countNonZero(filtered);
    if (numPositive >= min_positive) {
      detected_color = "blue";
    }

    // Set the response
    response->color = detected_color;
    RCLCPP_INFO(this->get_logger(), "Detected color: %s", detected_color.c_str());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetector>());
  rclcpp::shutdown();
  return 0;
}
