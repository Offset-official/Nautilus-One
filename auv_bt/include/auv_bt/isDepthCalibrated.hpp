#ifndef AUV_BT_ISDEPTHCALIBRATED_HPP_
#define AUV_BT_ISDEPTHCALIBRATED_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class IsDepthCalibrated : public BT::ConditionNode {
public:
  explicit IsDepthCalibrated(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts() {
    return BT::PortsList({BT::InputPort<double>("depth_threshold")});
  }

  void depth_callback(std_msgs::msg::Float64::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
  std_msgs::msg::Float64::UniquePtr last_depth_;
};

#endif
