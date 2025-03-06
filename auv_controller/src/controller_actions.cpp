#include "auv_controller/dumb_controller.hpp"
#include "auv_interfaces/action/depth_descent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using DepthDescent = auv_interfaces::action::DepthDescent;
using GoalHandleDepthDescent = rclcpp_action::ServerGoalHandle<DepthDescent>;

rclcpp_action::GoalResponse
DumbController::handle_goal(const rclcpp_action::GoalUUID &uuid,
                            std::shared_ptr<const DepthDescent::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with targe depth: %.2f meters",
              goal->target_depth);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DumbController::handle_cancel(
    const std::shared_ptr<GoalHandleDepthDescent> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DumbController::handle_accepted(
    const std::shared_ptr<GoalHandleDepthDescent> goal_handle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&DumbController::execute, this, _1), goal_handle}
      .detach();
}

void DumbController::execute(
    const std::shared_ptr<GoalHandleDepthDescent> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(500ms);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<DepthDescent::Feedback>();
  auto &depth = feedback->current_depth;
  auto result = std::make_shared<DepthDescent::Result>();

  this->target_depth_ = goal->target_depth;
  this->depth_reached_ = false;

  // invoke the led color
  auto color_request =
      std::make_shared<auv_interfaces::srv::SetColor::Request>();
  color_request->color = "#800080";

  auto req_= led_color_client_->async_send_request(color_request);
  req_.wait();

  while (!this->depth_reached_ && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      result->final_depth = this->current_depth_;
      this->target_depth_ = this->current_depth_;
      this->depth_reached_ = true;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(),
                  "Goal canceled. Holding depth at %.2f meters",
                  this->current_depth_);
      return;
    }
    depth = this->current_depth_;
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->final_depth = depth;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    color_request->color = "#ffffff";
    req_ = led_color_client_->async_send_request(color_request);
    req_.wait();
  }
}
