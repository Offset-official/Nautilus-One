#include <string>
#include <utility>

#include "auv_bt/Gate.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

Gate::Gate(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::ConditionNode(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  detections_sub_ =
      node_->create_subscription<auv_interfaces::msg::DetectionArray>(
          "/auv_camera_front/detections", 1,
          std::bind(&Gate::detections_callback, this, _1));

  last_reading_time_ = node_->now();
  last_computed_ratio = 0.0;
  last_num_leds = 1;
}

void Gate::detections_callback(
    auv_interfaces::msg::DetectionArray::UniquePtr msg) {
  last_detections_ = std::move(msg);

  if (last_detections_->detections.size() < 1) {
    return;
  }
  auto largest_size = 0.0;
  auto gate_center_x = 0;
  for (auto detection : last_detections_->detections) {
    auto x1 = detection.x1;
    auto y1 = detection.y1;
    auto x2 = detection.x2;
    auto y2 = detection.y2;

    // compute the area of the detection
    auto size = (x2 - x1) * (y2 - y1);
    if (size > largest_size) {
      largest_size = size;
      gate_center_x = (x2 + x1) / 2;
    }
  }

  auto largest_ratio = (double)largest_size / (double)screen_area;

  last_computed_ratio = largest_ratio;
  last_horizontal_error = screen_width - gate_center_x;
  // RCLCPP_INFO(node_->get_logger(), "found new gate with ratio %f",
  // last_computed_ratio);
}

int Gate::num_leds_to_turn_on(double current_size, double target_size) {

  auto percentage_diff = (current_size - target_size) / target_size;
  auto num_leds = max_leds * (1 - percentage_diff);
  return static_cast<int>(std::floor(num_leds));
}

BT::NodeStatus Gate::tick() {
  if (last_detections_ == nullptr) {
    return BT::NodeStatus::RUNNING;
  }

  if (last_detections_->detections.size() < 1) {
    return BT::NodeStatus::RUNNING;
  }

  double target_size = 0.5;
  getInput("target_size", target_size);

  auto num_leds = num_leds_to_turn_on(last_computed_ratio, target_size);
  setOutput("num", num_leds);

  setOutput("horizontal_error", last_horizontal_error);
  // RCLCPP_INFO(node_->get_logger(), "Num: %d, h_error: %d", num_leds,
  // last_horizontal_error);
  //  RCLCPP_INFO(node_->get_logger(), "target ratio: %f, h_error: %d",
  //  target_size, last_horizontal_error);
  if (last_computed_ratio > target_size) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<Gate>("Gate"); }
