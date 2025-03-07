#ifndef AUV_BT_ISARMED_HPP_
#define AUV_BT_ISARMED_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "mavros_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

class IsArmed : public BT::ConditionNode {
public:
  explicit IsArmed(const std::string &xml_tag_name,
                      const BT::NodeConfiguration &conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts() { return BT::PortsList({}); }

  void laser_callback(mavros_msgs::msg::State::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_sub_;
  mavros_msgs::msg::State::UniquePtr last_state_;
};

#endif
