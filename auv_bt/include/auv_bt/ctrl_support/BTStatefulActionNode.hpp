#ifndef AUV_BT__CTRL_SUPPORT__BTSTATEFULACTIONNODE_HPP_
#define AUV_BT__CTRL_SUPPORT__BTSTATEFULACTIONNODE_HPP_

#include <memory>
#include <string>
#include <functional>  // Add this near the other includes

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals; // NOLINT

template <class ActionT, class NodeT = rclcpp::Node>
class BtActionNode : public BT::StatefulActionNode {
public:
  BtActionNode(const std::string &xml_tag_name, const std::string &action_name,
               const BT::NodeConfiguration &conf)
      : BT::StatefulActionNode(xml_tag_name, conf), action_name_(action_name) {
    node_ =
        config().blackboard->template get<typename NodeT::SharedPtr>("node");

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ =
        typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtActionNode initialized",
                xml_tag_name.c_str());
  }

  BtActionNode() = delete;

  virtual ~BtActionNode() {}

  // Create instance of an action server
  void createActionClient(const std::string &action_name) {
    // Now that we have the ROS node to use, create the action client for this
    // BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name);

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" action server",
                action_name.c_str());
    action_client_->wait_for_action_server();
  }

  /* Any subclass of BtStatefulActionNode that accepts parameters
   * must provide a providedPorts method and call
   * providedBasicPorts in it.
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Action server name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  virtual void onNodeStart() {}

  virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }

  virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }

  /* Called when a the action is cancelled.
   * By default, the node will return SUCCESS.
   * The user may override it to return another value,
   * instead.
   */
  virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

  /* The main override required by a BT
   * StatefulActionNode
   */
  BT::NodeStatus onStart() {

    onNodeStart();

    // sending the action to the action server
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback = 
        std::bind(&BtActionNode::handleGoalResponse, this, std::placeholders::_1);

    auto future_goal_handle =
        action_client_->async_send_goal(goal_, send_goal_options);

    if (rclcpp::spin_until_future_complete(
            node_->get_node_base_interface(), future_goal_handle,
            server_timeout_) != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Failed to send action");
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the action server");
    }

    return BT::NodeStatus::RUNNING;
  }

  // If onStart() returned RUNNING, we will keep calling
  // this method until it return something different from RUNNING
  BT::NodeStatus onRunning() {
    if (!goal_result_available_) {
      return BT::NodeStatus::RUNNING;
    }

    switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return on_success();

    case rclcpp_action::ResultCode::ABORTED:
      return on_aborted();

    case rclcpp_action::ResultCode::CANCELED:
      return on_cancelled();

    default:
      throw std::logic_error("BtActionNode::Tick: invalid status value");
    }
  }

  void onHalted() {
    // we want to cancel the goal if it is running
    if (!goal_result_available_) {
      RCLCPP_INFO(node_->get_logger(), "!! recieved some goal update !!");
    }
  }

protected:
  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  typename NodeT::SharedPtr node_;

  std::chrono::milliseconds server_timeout_ = 10s;

  void handleGoalResponse(const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult &result) {
    // TODO(#1652): a work around until rcl_action interface is updated
    // if goal ids are not matched, the older goal call this callback so
    // ignore the result if matched, it must be processed (including aborted)
    RCLCPP_INFO(node_->get_logger(), "!! recieved some goal update !!");
    if (this->goal_handle_->get_goal_id() == result.goal_id) {
      goal_result_available_ = true;
      result_ = result;
    } else {
      RCLCPP_INFO(node_->get_logger(), "!! goal id mismatch !!");
    }
  }
};

#endif
