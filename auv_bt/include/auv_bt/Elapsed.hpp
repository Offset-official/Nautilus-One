#ifndef BT_ELAPSED_HPP_
#define BT_ELAPSED_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class Elapsed : public BT::ConditionNode {
public:
  explicit Elapsed(const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts() {
    return BT::PortsList({BT::InputPort<int>("seconds")});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
};

#endif
