#ifndef AUV_BT__MOVE_HPP_
#define AUV_BT__MOVE_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auv_bt/ctrl_support/BTStatefulActionNode.hpp"
#include "auv_interfaces/action/depth_descent.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class Descend : public BtActionNode<auv_interfaces::action::DepthDescent>
{
public:
  explicit Descend(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void onNodeStart() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("target_depth")
    };
  }
};

#endif
