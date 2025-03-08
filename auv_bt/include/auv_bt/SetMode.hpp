#ifndef AUV_BT__MODE_HPP_
#define AUV_BT__MODE_HPP_

#include <string>

#include "auv_bt/ctrl_support/BTServiceNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mavros_msgs/srv/command_bool.hpp"

class SetMode : public BtServiceNode<mavros_msgs::srv::CommandBool> {
public:
  explicit SetMode(const std::string &xml_tag_name,
                       const std::string &action_name,
                       const BT::NodeConfiguration &conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts() { return {BT::InputPort<bool>("arm")}; }
};

#endif
