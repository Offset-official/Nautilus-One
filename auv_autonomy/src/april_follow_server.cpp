#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <math.h>
#include <memory>
#include <thread>

using namespace std::placeholders;
using AprilTagDetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
class AprilFollowActionServer : public rclcpp::Node {
public:
  AprilFollowActionServer() : Node("april_follow_action_server") {

    this->declare_parameter("screen_width", 640);
    this->declare_parameter("screen_height", 480);

    img_width_ = this->get_parameter("screen_width").as_int();
    img_height_ = this->get_parameter("screen_height").as_int();

    screen_area_ = img_width_ * img_height_;

    apriltag_subscription_ = this->create_subscription<AprilTagDetectionArray>(
        "detections", 10,
        std::bind(&AprilFollowActionServer::april_topic_callback, this, _1));
  }

private:
  rclcpp::Subscription<AprilTagDetectionArray>::SharedPtr
      apriltag_subscription_;

  uint32_t img_width_;
  uint32_t img_height_;
  int screen_area_;
  uint16_t target_tag_id_;
  double screen_coverage_ratio_;
  double tag_x_;

  void april_topic_callback(const AprilTagDetectionArray &msg) {
    for (auto tag : msg.detections) {
      if (tag.id == target_tag_id_) {
        tag_x_ = tag.centre.x;

        auto x1 = tag.corners[0].x;
        auto y1 = tag.corners[0].y;
        auto x2 = tag.corners[2].x;
        auto y2 = tag.corners[2].y;

        // compute the size of the rectangle
        auto x_mag = abs(x1 - x2);
        auto y_mag = abs(y1 - y2);
        auto tag_area = x_mag * y_mag;
        screen_coverage_ratio_ = tag_area / (double) screen_area_;

        RCLCPP_INFO(this->get_logger(), "screen area: %d & tag area: %f", screen_area_,tag_area);
        RCLCPP_INFO(this->get_logger(), "Tag %d -> coverage: %.2f", tag.id,
                    screen_coverage_ratio_);
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilFollowActionServer>());
  rclcpp::shutdown();
  return 0;
}
