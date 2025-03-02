#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <rclcpp/qos.hpp>
#include "auv_interfaces/action/depth_descent.hpp"
#include "auv_interfaces/srv/angle_correction.hpp"  // New service header
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using DepthDescent = auv_interfaces::action::DepthDescent;
using GoalHandleDepthDescent = rclcpp_action::ServerGoalHandle<DepthDescent>;
using namespace std::placeholders;

class DumbController : public rclcpp::Node
{
public:
  DumbController() : Node("dumb_controller")
  {
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
    double_param_desc.type =
        rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

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

    this->declare_parameter("yaw_p_gain", 10.0, double_param_desc);  // PWM change per degree of error
    this->declare_parameter("yaw_threshold", 2.0, double_param_desc); // 2 degrees threshold

    // Get PWM parameters
    neutral_pwm_ = this->get_parameter("neutral_pwm").as_int();
    surge_low_pwm_change_ =
        this->get_parameter("surge_low_pwm_change").as_int();
    surge_medium_pwm_change_ =
        this->get_parameter("surge_medium_pwm_change").as_int();
    surge_high_pwm_change_ =
        this->get_parameter("surge_high_pwm_change").as_int();

    yaw_low_pwm_change_ = this->get_parameter("yaw_low_pwm_change").as_int();
    yaw_medium_pwm_change_ =
        this->get_parameter("yaw_medium_pwm_change").as_int();
    yaw_high_pwm_change_ = this->get_parameter("yaw_high_pwm_change").as_int();

    heave_low_pwm_change_ =
        this->get_parameter("heave_low_pwm_change").as_int();
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
        std::bind(&DumbController::twist_callback, this,
                  std::placeholders::_1));

    // for getting current depth from sensor
    current_depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/current_depth", 10,
        std::bind(&DumbController::current_depth_callback, this,
                  std::placeholders::_1));
    rclcpp::QoS qos_profile(10);       // Queue size of 10
    qos_profile.best_effort();         // Match the BEST_EFFORT reliability
    qos_profile.durability_volatile(); // Match the VOLATILE durability

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "mavros/imu/data",
        qos_profile,
        std::bind(&DumbController::imu_callback, this, std::placeholders::_1));

    // Set up publishers
    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
        "/mavros/rc/override", 10);

    // Set up arm client
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");
    mode_client_ =
        this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        
    // Set up angle correction service
    angle_correction_srv_ = this->create_service<auv_interfaces::srv::AngleCorrection>(
        "angle_correction",
        std::bind(&DumbController::handle_angle_correction, this, _1, _2));

    // Arm the vehicle
    arm_vehicle(true);
    set_mode("MANUAL");

    // Set up timer
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_rate),
        std::bind(&DumbController::timer_callback, this));
  }

private:
  void timer_callback()
  {
    control_callback();
    publish_rc_override();
    

  }

  // Calculate the smallest angle between two yaw values (accounting for wraparound)
  double calculate_yaw_difference(double current, double stored)
  {
    double diff = current - stored;
    
    // Normalize to [-180, 180]
    if (diff > 180.0) {
      diff -= 360.0;
    } else if (diff < -180.0) {
      diff += 360.0;
    }
    
    return diff;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double roll, pitch, yaw;
    quaternion_to_euler(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w, roll, pitch, yaw);
    
    // Store current yaw for use in angle correction
    current_yaw_ = yaw;

    // RCLCPP_INFO(this->get_logger(), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
  }

  // Handle the angle correction service call
  void handle_angle_correction(
      const std::shared_ptr<auv_interfaces::srv::AngleCorrection::Request> request,
      std::shared_ptr<auv_interfaces::srv::AngleCorrection::Response> response)
  {
    angle_correction_enabled_ = request->enable;
    
    if (angle_correction_enabled_) {
      // Store the current yaw angle when enabled
      stored_yaw_ = current_yaw_;
      RCLCPP_INFO(this->get_logger(), "Angle correction enabled, stored yaw: %.2f degrees", stored_yaw_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Angle correction disabled");
    }
    
    response->success = true;
    response->message = angle_correction_enabled_ ? 
                         "Angle correction enabled, reference yaw stored" : 
                         "Angle correction disabled";
  }

  void quaternion_to_euler(double x, double y, double z, double w, double &roll, double &pitch, double &yaw)
  {
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
      pitch = std::copysign(90.0, sinp); // Use 90 degrees if out of range
    else
      pitch = std::asin(sinp) * 180.0 / M_PI;

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
  }
  void current_depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_depth_ = msg->data;
  }

  void set_mode(const std::string &mode)
  {
    if (!mode_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;

    auto callback =
        [this,
         mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
    {
      auto result = future.get();
      if (result->mode_sent)
      {
        RCLCPP_INFO(this->get_logger(), "Successfully set mode to: %s",
                    mode.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set mode to: %s",
                     mode.c_str());
      }
    };

    mode_client_->async_send_request(request, callback);
  }

  void twist_callback(const geometry_msgs::msg::Twist &msg)
  {
    target_vel_surge = msg.linear.y;
    target_vel_yaw = msg.angular.z;
    target_vel_heave = msg.linear.z;
  }

void control_callback()
{
    double depth_error = target_depth_ - current_depth_;
    double threshold = this->get_parameter("depth_threshold").as_double();
    double p_gain = this->get_parameter("depth_p_gain").as_double();

    // Standard depth control logic
    if (!depth_reached_ && std::abs(depth_error) <= threshold)
    {
        RCLCPP_INFO(this->get_logger(), "Target depth %.2f reached",
                    target_depth_);
        target_vel_heave = 0.0;
        depth_reached_ = true;
    }
    else if (!depth_reached_)
    {
        target_vel_heave = depth_error * p_gain;
        RCLCPP_INFO(this->get_logger(),
                    "Depth control - Current: %.2f, Target: %.2f, Error: %.2f, "
                    "Command: %.2f",
                    current_depth_, target_depth_, depth_error, target_vel_heave);
    }

    if (std::abs(depth_error) > threshold)
    {
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
            yaw_pwm_ = std::max(neutral_pwm_-yaw_high_pwm_change_, std::min(neutral_pwm_+yaw_high_pwm_change_, yaw_pwm_));
            
            RCLCPP_INFO(this->get_logger(),
                      "Angle-based yaw correction - Current: %.2f, Target: %.2f, Error: %.2f, "
                      "PWM: %d",
                      current_yaw_, stored_yaw_, yaw_error, yaw_pwm_);
        } else {
            // Within threshold, no correction needed
            yaw_pwm_ = neutral_pwm_;
            RCLCPP_INFO(this->get_logger(), "Yaw within threshold (%.2f°), no correction needed", yaw_threshold);
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
  int calculatePWM(float target_vel, const float high_cut_off,
                   const float medium_cut_off, const float low_cut_off,
                   const int high_pwm_change, const int medium_pwm_change,
                   const int low_pwm_change)
  {
    float abs_target = abs(target_vel);
    int pwm_change = 0;
    int pwm = 0;

    if (abs_target >= high_cut_off)
    {
      pwm_change = high_pwm_change;
    }
    else if (abs_target >= medium_cut_off)
    {
      pwm_change = medium_pwm_change;
    }
    else if (abs_target >= low_cut_off)
    {
      pwm_change = low_pwm_change;
    }

    pwm = neutral_pwm_ + (target_vel >= 0 ? pwm_change : -pwm_change);
    return pwm;
  }
  void publish_rc_override()
  {
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

  void arm_vehicle(bool arm)
  {
    while (!arm_client_->wait_for_service(1s) && rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for arm service to be available...");
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;

    auto future = arm_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Vehicle %s successfully",
                  arm ? "armed" : "disarmed");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call arm service");
    }
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const DepthDescent::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with targe depth: %.2f meters",
                goal->target_depth);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDepthDescent> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleDepthDescent> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&DumbController::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleDepthDescent> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(500ms);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DepthDescent::Feedback>();
    auto &depth = feedback->current_depth;
    auto result = std::make_shared<DepthDescent::Result>();

    this->target_depth_ = goal->target_depth;
    this->depth_reached_ = false;

    while (!this->depth_reached_ && rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
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
    if (rclcpp::ok())
    {
      result->final_depth = depth;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  int heave_pwm_ = 1500;
  int surge_pwm_ = 1500;
  int yaw_pwm_ = 1500;
  double velocity_surge, velocity_yaw, velocity_heave;
  double target_vel_surge, target_vel_yaw, target_vel_heave;

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
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;

  // Service client
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  
  // Angle correction service
  rclcpp::Service<auv_interfaces::srv::AngleCorrection>::SharedPtr angle_correction_srv_;
  
  // Action Server
  rclcpp_action::Server<DepthDescent>::SharedPtr action_server_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<DumbController>();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}