#ifndef AUV_BT_GATE_HPP_
#define AUV_BT_GATE_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "auv_interfaces/msg/detection_array.hpp"
#include "rclcpp/rclcpp.hpp"

class Gate : public BT::ConditionNode {
public:
  explicit Gate(const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<double>("target_size"),
        BT::InputPort<std::string>("mode"), // not working right now should either be max or min
        BT::OutputPort<int>("num"), // number of leds which should be lit up
        BT::OutputPort<int>("horizontal_error"), // the horizontal deviation from the center
    });
  }

  void detections_callback(auv_interfaces::msg::DetectionArray::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr
      detections_sub_;
  auv_interfaces::msg::DetectionArray::UniquePtr last_detections_;
  double last_computed_ratio;
  int last_horizontal_error;
  int last_num_leds;
  const int max_leds = 20;
  const double screen_width = 720;
  const double screen_height = 1280;
  const double screen_area = screen_width * screen_height;

  int num_leds_to_turn_on(double, double);
};

#endif
