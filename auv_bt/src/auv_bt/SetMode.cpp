#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "auv_bt/SetMode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mavros_msgs/srv/command_bool.hpp"

SetMode::SetMode(const std::string &xml_tag_name,
                 const std::string &action_name,
                 const BT::NodeConfiguration &conf)
    : BtServiceNode<mavros_msgs::srv::CommandBool>(xml_tag_name, action_name,
                                                   conf) {}

void SetMode::on_tick() {
  bool arm;
  getInput("arm", arm);

  request_->value = arm;
}

BT::NodeStatus SetMode::on_success() {
  RCLCPP_INFO(node_->get_logger(), "arming request sent successfully");

  // the condition node should return true not us
  return BT::NodeStatus::RUNNING;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<SetMode>(name, "/mavros/cmd/arming", config);
  };

  factory.registerBuilder<SetMode>("SetMode", builder);
}
