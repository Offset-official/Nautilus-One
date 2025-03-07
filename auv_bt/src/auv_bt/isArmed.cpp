#include <string>
#include <utility>

#include "auv_bt/isArmed.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

IsArmed::IsArmed(const std::string &xml_tag_name,
                 const BT::NodeConfiguration &conf)
    : BT::ConditionNode(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  mavros_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
      "/input_scan", 100, std::bind(&IsArmed::laser_callback, this, _1));

  last_reading_time_ = node_->now();
}

void IsArmed::laser_callback(mavros_msgs::msg::State::UniquePtr msg) {
  last_state_ = std::move(msg);
}

BT::NodeStatus IsArmed::tick() {
  if (last_state_ == nullptr) {
    return BT::NodeStatus::FAILURE;
  }

  if (last_state_->armed) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<IsArmed>("IsArmed"); }
