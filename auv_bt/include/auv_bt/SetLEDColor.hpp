#ifndef AUV_BT__LED_HPP_
#define AUV_BT__LED_HPP_

#include <string>

#include "auv_bt/ctrl_support/BTServiceNode.hpp"
#include "auv_interfaces/srv/set_color.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class SetLEDColor : public BtServiceNode<auv_interfaces::srv::SetColor> {
public:
  explicit SetLEDColor(const std::string &xml_tag_name,
                   const std::string &action_name,
                   const BT::NodeConfiguration &conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("color"),
            BT::InputPort<int>("num")};
  }
};

#endif
