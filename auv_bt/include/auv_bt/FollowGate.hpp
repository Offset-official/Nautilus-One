#ifndef AUV_BT_FOLLOW_HPP_
#define AUV_BT_FOLLOW_HPP_

#include <string>

#include "auv_interfaces/msg/detection_array.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class FollowGate : public BT::ActionNodeBase {
public:
  explicit FollowGate(const std::string &xml_tag_name,
                      const BT::NodeConfiguration &conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts() {
    return BT::PortsList({BT::InputPort<double>("forward_speed"),
                          BT::InputPort<double>("turn_speed"),
                          BT::InputPort<int>("horizontal_error"),
                          BT::InputPort<int>("error_threshold")});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr
      detections_sub_;
};

#endif
