#ifndef AUV_BT__CALIBRATE_DEPTH_HPP_
#define AUV_BT__CALIBRATE_DEPTH_HPP_

#include <string>

#include "auv_bt/ctrl_support/BTServiceNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_srvs/srv/trigger.hpp"

class CalibrateDepth : public BtServiceNode<std_srvs::srv::Trigger> {
public:
  explicit CalibrateDepth(const std::string &xml_tag_name,
                          const std::string &action_name,
                          const BT::NodeConfiguration &conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts() { return {}; }
};

#endif
