#include <string>
#include <utility>

#include "auv_bt/isDepthCalibrated.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

IsDepthCalibrated::IsDepthCalibrated(const std::string &xml_tag_name,
                                     const BT::NodeConfiguration &conf)
    : BT::ConditionNode(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  depth_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/current_depth", 5,
      std::bind(&IsDepthCalibrated::depth_callback, this, _1));

  last_reading_time_ = node_->now();
}

void IsDepthCalibrated::depth_callback(std_msgs::msg::Float64::UniquePtr msg) {
  last_depth_ = std::move(msg);
}

BT::NodeStatus IsDepthCalibrated::tick() {
  if (last_depth_ == nullptr) {
    return BT::NodeStatus::FAILURE;
  }

  double depth_threshold = 0.0;
  getInput("depth_threshold", depth_threshold);
  auto last_depth_value = last_depth_->data;
  if (abs(0 - last_depth_value) <= depth_threshold) {

    RCLCPP_INFO(node_->get_logger(), "depth calibrated to %f",last_depth_value);

    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<IsDepthCalibrated>("IsDepthCalibrated");
}
