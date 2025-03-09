#include <iostream>
#include <string>

#include "auv_bt/FollowGate.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "auv_interfaces/msg/detection_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

FollowGate::FollowGate(const std::string &xml_tag_name,
                       const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);
}

BT::NodeStatus FollowGate::tick() {

  double forward_speed = 0.5;
  getInput("forward_speed", forward_speed);

  double turn_speed = 0.5;
  getInput("turn_speed", turn_speed);

  int horizontal_error = 0;
  getInput("horizontal_error", horizontal_error);

  int error_threshold = 5;
  getInput("error_threshold", error_threshold);

  geometry_msgs::msg::Twist vel_msgs;

  RCLCPP_INFO(node_->get_logger(), "receieved h_error: %d", horizontal_error);
  if (horizontal_error > error_threshold) {
    // need to correct the heading
    vel_msgs.angular.z = horizontal_error > 0 ? -turn_speed : turn_speed;
    RCLCPP_INFO(node_->get_logger(), "correcting yaw for error: %d",
                horizontal_error);

  } else {
    vel_msgs.linear.y = forward_speed;
    RCLCPP_INFO(node_->get_logger(), "heading straight with %f",
                vel_msgs.linear.y);
  }

  vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;
}

void FollowGate::halt() {
  RCLCPP_INFO(node_->get_logger(), "FollowGate node halted. Sending 0 vel");
  geometry_msgs::msg::Twist vel_msgs;
  vel_pub_->publish(vel_msgs);
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<FollowGate>("FollowGate");
}
