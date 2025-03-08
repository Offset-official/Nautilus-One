#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>

#include "auv_interfaces/action/depth_descent.hpp"
#include "auv_interfaces/srv/angle_correction.hpp"
#include "auv_interfaces/srv/set_color.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using DepthDescent = auv_interfaces::action::DepthDescent;
using GoalHandleDepthDescent = rclcpp_action::ServerGoalHandle<DepthDescent>;
using namespace std::placeholders;

class DumbController : public rclcpp::Node {
public:
  DumbController();

private:
  void reset_velocities();
  rclcpp::TimerBase::SharedPtr velocity_reset_timer_;
  bool velocity_command_active_ = false;

  void send_calibration_request();
  void calibration_callback(
      rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  void timer_callback();
  double calculate_yaw_difference(double current, double stored);
  void compass_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void handle_angle_correction(
      const std::shared_ptr<auv_interfaces::srv::AngleCorrection::Request>
          request,
      std::shared_ptr<auv_interfaces::srv::AngleCorrection::Response> response);
  void soft_arm(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void current_depth_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void set_mode(const std::string &mode);
  void twist_callback(const geometry_msgs::msg::Twist &msg);
  void control_callback();
  int calculatePWM(float target_vel, const float high_cut_off,
                   const float medium_cut_off, const float low_cut_off,
                   const int high_pwm_change, const int medium_pwm_change,
                   const int low_pwm_change);
  void publish_rc_override();
  void arm_vehicle(bool arm);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const DepthDescent::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDepthDescent> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleDepthDescent> goal_handle);

  void execute(const std::shared_ptr<GoalHandleDepthDescent> goal_handle);

  int heave_pwm_ = 1500;
  int surge_pwm_ = 1500;
  int yaw_pwm_ = 1500;
  double velocity_surge, velocity_yaw, velocity_heave;
  double target_vel_surge, target_vel_yaw, target_vel_heave;

  bool soft_arm_ = false;

  int neutral_pwm_;
  int surge_low_pwm_change_;
  int surge_medium_pwm_change_;
  int surge_high_pwm_change_;
  int yaw_low_pwm_change_;
  int yaw_medium_pwm_change_;
  int yaw_high_pwm_change_;
  int heave_low_pwm_change_;
  int heave_medium_pwm_change_;
  int heave_high_pwm_change_;

  double surge_low_cut_off;
  double surge_medium_cut_off;
  double surge_high_cut_off;
  double yaw_low_cut_off;
  double yaw_medium_cut_off;
  double yaw_high_cut_off;
  double heave_low_cut_off;
  double heave_medium_cut_off;
  double heave_high_cut_off;

  double current_depth_;
  double target_depth_;
  bool depth_reached_;

  // Angle correction variables
  double current_yaw_;
  double stored_yaw_;
  bool angle_correction_enabled_;

  // Time tracking
  rclcpp::Time last_time_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      twist_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_sub_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;

  // Service client
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibration_client_;
  rclcpp::Client<auv_interfaces::srv::SetColor>::SharedPtr led_color_client_;

  // Angle correction service
  rclcpp::Service<auv_interfaces::srv::AngleCorrection>::SharedPtr
      angle_correction_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr soft_arm_srv_;

  // Action Server
  rclcpp_action::Server<DepthDescent>::SharedPtr action_server_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};