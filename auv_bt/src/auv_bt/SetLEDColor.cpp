#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "auv_bt/SetLEDColor.hpp"

#include "auv_interfaces/srv/set_color.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

SetLEDColor::SetLEDColor(const std::string &xml_tag_name,
                         const std::string &action_name,
                         const BT::NodeConfiguration &conf)
    : BtServiceNode<auv_interfaces::srv::SetColor>(xml_tag_name, action_name,
                                                   conf) {}

void SetLEDColor::on_tick() {
  std::string color;
  int num;
  getInput("color", color);
  getInput("num", num);

  if (num > 20)
    num = 20;

  request_->color = color;
  request_->color_count = num;
}

BT::NodeStatus SetLEDColor::on_success() {
  RCLCPP_INFO(node_->get_logger(), "/set_color Suceeded");

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<SetLEDColor>(name, "/set_color", config);
  };

  factory.registerBuilder<SetLEDColor>("SetLEDColor", builder);
}
