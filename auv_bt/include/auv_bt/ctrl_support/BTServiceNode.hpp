#ifndef AUV_BT__CTRL_SUPPORT__BTSERVICENODE_HPP_
#define AUV_BT__CTRL_SUPPORT__BTSERVICENODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // NOLINT

template <class ServiceT, class NodeT = rclcpp::Node>
class BtServiceNode : public BT::ActionNodeBase {
public:
  BtServiceNode(const std::string &xml_tag_name,
                const std::string &service_name,
                const BT::NodeConfiguration &conf)
      : BT::ActionNodeBase(xml_tag_name, conf), service_name_(service_name) {
    node_ =
        config().blackboard->template get<typename NodeT::SharedPtr>("node");

    server_timeout_ = 1s;

    // Initialize the request and response
    request_ = std::make_shared<typename ServiceT::Request>();

    std::string remapped_service_name;
    if (getInput("server_name", remapped_service_name)) {
      service_name_ = remapped_service_name;
    }
    createServiceClient(service_name_);

    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtServiceNode initialized",
                xml_tag_name.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode() {}

  // Create instance of a service client
  void createServiceClient(const std::string &service_name) {
    client_ = node_->template create_client<ServiceT>(service_name);

    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" service",
                service_name.c_str());
    if (!client_->wait_for_service(server_timeout_)) {
      RCLCPP_ERROR(node_->get_logger(), "Service %s not available",
                   service_name.c_str());
      throw std::runtime_error("Service not available");
    }
  }

  // Any subclass of BtServiceNode that accepts parameters must provide a
  // providedPorts method and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("server_name", "Service name"),
        BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  // Hook methods
  virtual void on_tick() {}
  virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }
  virtual BT::NodeStatus on_failed() { return BT::NodeStatus::FAILURE; }

  // The main override required by a BT action
  BT::NodeStatus tick() override {
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING);

      // Prepare the request
      on_tick();

      if (!client_->service_is_ready()) {
        RCLCPP_ERROR(node_->get_logger(), "Service %s not ready",
                     service_name_.c_str());
        return BT::NodeStatus::FAILURE;
      }

      // Send the request and wait for the response
      auto future_result = client_->async_send_request(request_);

      if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                             future_result, server_timeout_) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s",
                     service_name_.c_str());
        return on_failed();
      }

      response_ = future_result.get();
      return on_success();
    }

    return BT::NodeStatus::RUNNING;
  }

  void halt() override { setStatus(BT::NodeStatus::IDLE); }

protected:
  std::string service_name_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

  // Service request and response
  typename std::shared_ptr<typename ServiceT::Request> request_;
  typename std::shared_ptr<typename ServiceT::Response> response_;

  // The node that will be used for any ROS operations
  typename NodeT::SharedPtr node_;

  // The timeout value while waiting for response from a server
  std::chrono::milliseconds server_timeout_;
};

#endif // AUV_BT__CTRL_SUPPORT__BTSERVICENODE_HPP_
