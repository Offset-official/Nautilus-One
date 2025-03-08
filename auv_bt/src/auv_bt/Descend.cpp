#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "auv_bt/Descend.hpp"

#include "auv_interfaces/action/depth_descent.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

Descend::Descend(const std::string &xml_tag_name,
                 const std::string &action_name,
                 const BT::NodeConfiguration &conf)
    : BtActionNode<auv_interfaces::action::DepthDescent>(xml_tag_name,
                                                         action_name, conf) {}

void Descend::on_tick() {
  double target_depth;
  getInput("target_depth", target_depth);

  goal_.target_depth = target_depth;
}

BT::NodeStatus Descend::on_success() {
  RCLCPP_INFO(node_->get_logger(), "depth descent Suceeded");

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<Descend>(name, "depth_descent", config);
  };

  factory.registerBuilder<Descend>("Descend", builder);
}
