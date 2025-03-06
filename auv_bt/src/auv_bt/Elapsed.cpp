#include <iostream>
#include <string>

#include "auv_bt/Elapsed.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

Turn::Turn(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);
}

void Turn::halt() {}

BT::NodeStatus Turn::tick() {
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = 0.5;

  auto elapsed = node_->now() - start_time_;

  if (elapsed < 3s) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<Turn>("Turn"); }
