#include <iostream>
#include <string>

#include "auv_bt/Turn.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

Turn::Turn(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);
}

void Turn::halt() {
  RCLCPP_INFO(node_->get_logger(), "Turn node halted. Sending 0 vel");
  geometry_msgs::msg::Twist vel_msgs;
  vel_pub_->publish(vel_msgs);
}

BT::NodeStatus Turn::tick() {

  double speed = 0.5;
  getInput("speed", speed);

  geometry_msgs::msg::Twist vel_msgs;
  vel_msgs.angular.z = speed;
  vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) { factory.registerNodeType<Turn>("Turn"); }
