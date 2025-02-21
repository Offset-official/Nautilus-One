#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using std::placeholders::_1;

using namespace std::chrono_literals;

class ColorDetector : public rclcpp::Node {
public:
  ColorDetector() : Node("color_detector_debug") {

    image_sub_ = image_transport::create_subscription(
        this, "input_image",
        std::bind(&ColorDetector::image_callback, this, _1), "compressed",
        rclcpp::SensorDataQoS().get_rmw_qos_profile());

    // Initialize the trackbars for adjusting HSV values
    cv::namedWindow("Trackbars", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("H Min", "Trackbars", &h_min_, 180);
    cv::createTrackbar("S Min", "Trackbars", &s_min_, 255);
    cv::createTrackbar("V Min", "Trackbars", &v_min_, 255);
    cv::createTrackbar("H Max", "Trackbars", &h_max_, 180);
    cv::createTrackbar("S Max", "Trackbars", &s_max_, 255);
    cv::createTrackbar("V Max", "Trackbars", &v_max_, 255);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  image_transport::Subscriber image_sub_;

  // Trackbar variables for adjusting HSV values
  int h_min_ = 0, s_min_ = 0, v_min_ = 0;
  int h_max_ = 180, s_max_ = 255, v_max_ = 255;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    // Update the HSV filter ranges from the trackbars if debugging is enabled
    h_min_ = cv::getTrackbarPos("H Min", "Trackbars");
    s_min_ = cv::getTrackbarPos("S Min", "Trackbars");
    v_min_ = cv::getTrackbarPos("V Min", "Trackbars");
    h_max_ = cv::getTrackbarPos("H Max", "Trackbars");
    s_max_ = cv::getTrackbarPos("S Max", "Trackbars");
    v_max_ = cv::getTrackbarPos("V Max", "Trackbars");

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    };

    cv::Mat img_hsv;
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

    cv::Mat1b filtered;
    cv::inRange(img_hsv, cv::Scalar(h_min_, s_min_, v_min_),
                cv::Scalar(h_max_, s_max_, v_max_), filtered);

    auto numPositive = cv::countNonZero(filtered);
    RCLCPP_INFO(this->get_logger(),"Num positive: %d",numPositive);

    // Display the filtered image
    cv::imshow("Filtered Image", filtered);
    cv::waitKey(1);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetector>());
  rclcpp::shutdown();
  return 0;
}
