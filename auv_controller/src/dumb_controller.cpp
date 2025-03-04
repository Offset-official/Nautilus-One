#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float64.hpp>

#include "auv_controller/dumb_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;

DumbController::DumbController() : Node("dumb_controller") {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;

  // PWM parameters
  this->declare_parameter("neutral_pwm", 1500, param_desc);

  this->declare_parameter("surge_low_pwm_change", 30, param_desc);
  this->declare_parameter("surge_medium_pwm_change", 40, param_desc);
  this->declare_parameter("surge_high_pwm_change", 50, param_desc);

  this->declare_parameter("yaw_low_pwm_change", 40, param_desc);
  this->declare_parameter("yaw_medium_pwm_change", 40, param_desc);
  this->declare_parameter("yaw_high_pwm_change", 40, param_desc);

  this->declare_parameter("heave_low_pwm_change", 50, param_desc);
  this->declare_parameter("heave_medium_pwm_change", 50, param_desc);
  this->declare_parameter("heave_high_pwm_change", 50, param_desc);

  // Velocity threshold parameters
  rcl_interfaces::msg::ParameterDescriptor double_param_desc;
  double_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

  this->declare_parameter("surge_low_cut_off", 0.2, double_param_desc);
  this->declare_parameter("surge_medium_cut_off", 0.5, double_param_desc);
  this->declare_parameter("surge_high_cut_off", 0.8, double_param_desc);

  this->declare_parameter("yaw_low_cut_off", 0.2, double_param_desc);
  this->declare_parameter("yaw_medium_cut_off", 0.5, double_param_desc);
  this->declare_parameter("yaw_high_cut_off", 0.8, double_param_desc);

  this->declare_parameter("heave_low_cut_off", 0.05, double_param_desc);
  this->declare_parameter("heave_medium_cut_off", 0.3, double_param_desc);
  this->declare_parameter("heave_high_cut_off", 0.6, double_param_desc);

  this->declare_parameter("depth_p_gain", 1.0, double_param_desc);
  this->declare_parameter("depth_threshold", 0.025, double_param_desc);

  this->declare_parameter("publish_rate_ms", 50, param_desc);

  this->declare_parameter("yaw_p_gain", 10.0,
                          double_param_desc); // PWM change per degree of error
  this->declare_parameter("yaw_threshold", 2.0,
                          double_param_desc); // 2 degrees threshold

  // Get PWM parameters
  neutral_pwm_ = this->get_parameter("neutral_pwm").as_int();
  surge_low_pwm_change_ = this->get_parameter("surge_low_pwm_change").as_int();
  surge_medium_pwm_change_ =
      this->get_parameter("surge_medium_pwm_change").as_int();
  surge_high_pwm_change_ =
      this->get_parameter("surge_high_pwm_change").as_int();

  yaw_low_pwm_change_ = this->get_parameter("yaw_low_pwm_change").as_int();
  yaw_medium_pwm_change_ =
      this->get_parameter("yaw_medium_pwm_change").as_int();
  yaw_high_pwm_change_ = this->get_parameter("yaw_high_pwm_change").as_int();

  heave_low_pwm_change_ = this->get_parameter("heave_low_pwm_change").as_int();
  heave_medium_pwm_change_ =
      this->get_parameter("heave_medium_pwm_change").as_int();
  heave_high_pwm_change_ =
      this->get_parameter("heave_high_pwm_change").as_int();

  // Get velocity threshold parameters
  surge_low_cut_off = this->get_parameter("surge_low_cut_off").as_double();
  surge_medium_cut_off =
      this->get_parameter("surge_medium_cut_off").as_double();
  surge_high_cut_off = this->get_parameter("surge_high_cut_off").as_double();

  yaw_low_cut_off = this->get_parameter("yaw_low_cut_off").as_double();
  yaw_medium_cut_off = this->get_parameter("yaw_medium_cut_off").as_double();
  yaw_high_cut_off = this->get_parameter("yaw_high_cut_off").as_double();

  heave_low_cut_off = this->get_parameter("heave_low_cut_off").as_double();
  heave_medium_cut_off =
      this->get_parameter("heave_medium_cut_off").as_double();
  heave_high_cut_off = this->get_parameter("heave_high_cut_off").as_double();

  int publish_rate = this->get_parameter("publish_rate_ms").as_int();

  // Initialize state variables
  velocity_surge = 0.0;
  velocity_yaw = 0.0;
  velocity_heave = 0.0;
  target_vel_surge = 0.0;
  target_vel_yaw = 0.0;
  target_vel_heave = 0.0;

  current_depth_ = 0.0;
  target_depth_ = 0.0;

  // Initialize angle correction variables
  current_yaw_ = 0.0;
  stored_yaw_ = 0.0;
  angle_correction_enabled_ = false;

  // Log initialization
  RCLCPP_INFO(this->get_logger(), "Initializing DumbController");

  // setup action server
  action_server_ = rclcpp_action::create_server<DepthDescent>(
      this, "depth_descent",
      std::bind(&DumbController::handle_goal, this, _1, _2),
      std::bind(&DumbController::handle_cancel, this, _1),
      std::bind(&DumbController::handle_accepted, this, _1));

  // Set up subscriptions
  // for setting target velocities for intelligence
  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&DumbController::twist_callback, this, std::placeholders::_1));

  // for getting current depth from sensor
  current_depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/current_depth", 10,
      std::bind(&DumbController::current_depth_callback, this,
                std::placeholders::_1));

  auto qos_profile =
      rclcpp::QoS(10)
          .reliability(
              rclcpp::ReliabilityPolicy::BestEffort) // Change from BEST_EFFORT
                                                     // to RELIABLE
          .durability(rclcpp::DurabilityPolicy::Volatile)   // Keep VOLATILE
          .liveliness(rclcpp::LivelinessPolicy::Automatic); // Keep AUTOMATIC

  compass_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/mavros/global_position/compass_hdg", qos_profile,
      std::bind(&DumbController::compass_callback, this,
                std::placeholders::_1));

  // Set up publishers
  rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
      "/mavros/rc/override", 10);

  // Set up arm client
  arm_client_ =
      this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
  mode_client_ =
      this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  calibration_client_ =
      this->create_client<std_srvs::srv::Trigger>("/calibrate_depth_sensor");
  // Set up angle correction service
  angle_correction_srv_ =
      this->create_service<auv_interfaces::srv::AngleCorrection>(
          "angle_correction",
          std::bind(&DumbController::handle_angle_correction, this, _1, _2));
  soft_arm_srv_ = this->create_service<std_srvs::srv::SetBool>("soft_arm",
          std::bind(&DumbController::soft_arm, this, _1, _2));

  // Arm the vehicle
  send_calibration_request();
  arm_vehicle(true);
  set_mode("MANUAL");

  // Set up timer
  publish_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(publish_rate),
                              std::bind(&DumbController::timer_callback, this));
}

void DumbController::timer_callback() {
  control_callback();
  publish_rc_override();
}

// Calculate the smallest angle between two yaw values (accounting for
// wraparound)
double DumbController::calculate_yaw_difference(double current, double stored) {
  double diff = current - stored;

  // Normalize to [-180, 180]
  if (diff > 180.0) {
    diff -= 360.0;
  } else if (diff < -180.0) {
    diff += 360.0;
  }

  return diff;
}

void DumbController::compass_callback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  current_yaw_ = msg->data;

  RCLCPP_INFO(this->get_logger(), "GOT DATA FROM CAOMPASS");
  RCLCPP_INFO(this->get_logger(), "Current Heading: %.2f degrees", msg->data);
}

void DumbController::current_depth_callback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  current_depth_ = msg->data;
}

void DumbController::twist_callback(const geometry_msgs::msg::Twist &msg) {
  target_vel_surge = msg.linear.y;
  target_vel_yaw = msg.angular.z;
  target_vel_heave = msg.linear.z;
}

void DumbController::control_callback() {
  double depth_error = target_depth_ - current_depth_;
  double threshold = this->get_parameter("depth_threshold").as_double();
  double p_gain = this->get_parameter("depth_p_gain").as_double();

  // Standard depth control logic
  if (!depth_reached_ && std::abs(depth_error) <= threshold) {
    RCLCPP_INFO(this->get_logger(), "Target depth %.2f reached", target_depth_);
    target_vel_heave = 0.0;
    depth_reached_ = true;
  } else if (!depth_reached_) {
    target_vel_heave = depth_error * p_gain;
    RCLCPP_INFO(this->get_logger(),
                "Depth control - Current: %.2f, Target: %.2f, Error: %.2f, "
                "Command: %.2f",
                current_depth_, target_depth_, depth_error, target_vel_heave);
  }

  if (std::abs(depth_error) > threshold) {
    depth_reached_ = false;
  }

  // Calculate heave and surge PWM values normally
  heave_pwm_ =
      calculatePWM(target_vel_heave, heave_high_cut_off, heave_medium_cut_off,
                   heave_low_cut_off, heave_high_pwm_change_,
                   heave_medium_pwm_change_, heave_low_pwm_change_);

  surge_pwm_ =
      calculatePWM(target_vel_surge, surge_high_cut_off, surge_medium_cut_off,
                   surge_low_cut_off, surge_high_pwm_change_,
                   surge_medium_pwm_change_, surge_low_pwm_change_);

  RCLCPP_INFO(this->get_logger(),
              "Angle Correction Mode State: %d, Heading: %.2f",
              angle_correction_enabled_, current_yaw_);
  // Yaw control - split between angle correction mode and normal mode
  if (angle_correction_enabled_) {
    // Calculate yaw error (smallest angle between current and stored yaw)
    double yaw_error = calculate_yaw_difference(current_yaw_, stored_yaw_);

    // Get yaw correction parameters
    double yaw_p_gain = this->get_parameter("yaw_p_gain").as_double();
    double yaw_threshold = this->get_parameter("yaw_threshold").as_double();

    // Only apply correction if error is above threshold
    if (std::abs(yaw_error) > yaw_threshold) {
      // Calculate PWM directly from angle error
      int yaw_correction = static_cast<int>(-yaw_error * yaw_p_gain);
      yaw_pwm_ = neutral_pwm_ + yaw_correction;

      // Ensure PWM stays within safe limits
      yaw_pwm_ =
          std::max(neutral_pwm_ - yaw_high_pwm_change_,
                   std::min(neutral_pwm_ + yaw_high_pwm_change_, yaw_pwm_));

      RCLCPP_INFO(this->get_logger(),
                  "Angle-based yaw correction - Target: %.2f, Error: %.2f, "
                  "PWM: %d",
                  stored_yaw_, yaw_error, yaw_pwm_);
    } else {
      // Within threshold, no correction needed
      yaw_pwm_ = neutral_pwm_;
      RCLCPP_INFO(this->get_logger(),
                  "Yaw within threshold (%.2f°), no correction needed",
                  yaw_threshold);
    }
  } else {
    // Normal operation - use velocity-based control for yaw
    yaw_pwm_ = calculatePWM(
        target_vel_yaw, yaw_high_cut_off, yaw_medium_cut_off, yaw_low_cut_off,
        yaw_high_pwm_change_, yaw_medium_pwm_change_, yaw_low_pwm_change_);
  }

  RCLCPP_INFO(this->get_logger(),
              "\n\tSurge | Yaw | Heave\nTarget_vel: [%.2f, %.2f, %.2f]\nPWM "
              "sent to thrusters: [%d, %d,%d]",
              target_vel_surge, target_vel_yaw, target_vel_heave, surge_pwm_,
              yaw_pwm_, heave_pwm_);
}

// TODO add custmized PWM for backward motion
int DumbController::calculatePWM(float target_vel, const float high_cut_off,
                                 const float medium_cut_off,
                                 const float low_cut_off,
                                 const int high_pwm_change,
                                 const int medium_pwm_change,
                                 const int low_pwm_change) {
  float abs_target = abs(target_vel);
  int pwm_change = 0;
  int pwm = 0;

  if (abs_target >= high_cut_off) {
    pwm_change = high_pwm_change;
  } else if (abs_target >= medium_cut_off) {
    pwm_change = medium_pwm_change;
  } else if (abs_target >= low_cut_off) {
    pwm_change = low_pwm_change;
  }

  pwm = neutral_pwm_ + (target_vel >= 0 ? pwm_change : -pwm_change);
  return pwm;
}

void DumbController::publish_rc_override() {
  auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();
  rc_out_msg.channels = {static_cast<unsigned short>(neutral_pwm_),
                         static_cast<unsigned short>(neutral_pwm_),
                         static_cast<unsigned short>(heave_pwm_),
                         static_cast<unsigned short>(yaw_pwm_),
                         static_cast<unsigned short>(surge_pwm_),
                         static_cast<unsigned short>(neutral_pwm_),
                         static_cast<unsigned short>(neutral_pwm_),
                         static_cast<unsigned short>(neutral_pwm_)};

  rc_pub_->publish(rc_out_msg);
  RCLCPP_DEBUG(this->get_logger(),
               "Published RC Override: Surge: %d, Yaw: %d, Heave: %d",
               surge_pwm_, yaw_pwm_, heave_pwm_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<DumbController>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
