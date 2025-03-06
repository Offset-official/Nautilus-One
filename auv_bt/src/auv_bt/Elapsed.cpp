#include <iostream>
#include <string>

#include "auv_bt/Elapsed.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

Elapsed::Elapsed(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::ConditionNode(xml_tag_name, conf) {
  config().blackboard->get("node", node_);
}

BT::NodeStatus Elapsed::tick() {
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
  }

  int seconds = 1.0;
  getInput("seconds", seconds);
  std::chrono::seconds _seconds{seconds};

  auto elapsed = node_->now() - start_time_;

  if (elapsed < _seconds) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<Elapsed>("Elapsed"); }
