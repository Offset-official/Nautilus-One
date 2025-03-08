#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "auv_bt/CalibrateDepth.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "mavros_msgs/srv/command_bool.hpp"

CalibrateDepth::CalibrateDepth(const std::string &xml_tag_name,
                               const std::string &action_name,
                               const BT::NodeConfiguration &conf)
    : BtServiceNode<std_srvs::srv::Trigger>(xml_tag_name, action_name, conf) {}

void CalibrateDepth::on_tick() {}

BT::NodeStatus CalibrateDepth::on_success() {
  RCLCPP_INFO(node_->get_logger(), "calibration request sent successfully");

  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<CalibrateDepth>(name, "/calibrate_depth_sensor", config);
  };

  factory.registerBuilder<CalibrateDepth>("CalibrateDepth", builder);
}
