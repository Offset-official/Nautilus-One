#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "auv_interfaces/action/depth_descent.hpp"
#include "auv_interfaces/srv/angle_correction.hpp"

using namespace std::chrono_literals;

class AUVController : public rclcpp::Node
{
public:
  AUVController()
  : Node("auv_controller"),
    current_mode_(""),
    angle_correction_enabled_(false)
  {
    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribers
    mode_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "mavros/state", 10, std::bind(&AUVController::mode_callback, this, std::placeholders::_1));

    // Service clients
    mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    angle_correction_client_ = this->create_client<auv_interfaces::srv::AngleCorrection>("/angle_correction");

    // Action client
    depth_action_client_ = rclcpp_action::create_client<auv_interfaces::action::DepthDescent>(
      this, "/depth_descent");

    // Timer for main control loop
    timer_ = this->create_wall_timer(500ms, std::bind(&AUVController::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "AUV Controller initialized");
  }

private:
  void mode_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_mode_ = msg->mode;
    RCLCPP_INFO(this->get_logger(), "Current mode: %s", current_mode_.c_str());
  }

  void set_mode(const std::string & mode)
  {
    if (!mode_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
      return;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;

    auto callback = [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        auto result = future.get();
        if (result->mode_sent) {
          RCLCPP_INFO(this->get_logger(), "Successfully set mode to: %s", mode.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to set mode to: %s", mode.c_str());
        }
      };

    mode_client_->async_send_request(request, callback);
  }

  void send_depth_action(float target_depth)
  {
    if (!depth_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Depth descent action server not available");
      return;
    }

    auto goal_msg = auv_interfaces::action::DepthDescent::Goal();
    goal_msg.target_depth = target_depth;

    RCLCPP_INFO(this->get_logger(), "Sending depth descent goal: %f", target_depth);

    auto send_goal_options = rclcpp_action::Client<auv_interfaces::action::DepthDescent>::SendGoalOptions();

    // Feedback callback
    send_goal_options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<auv_interfaces::action::DepthDescent>::SharedPtr,
        const std::shared_ptr<const auv_interfaces::action::DepthDescent::Feedback> feedback) {
        RCLCPP_INFO(
          this->get_logger(), "Depth descent feedback - Current depth: %f", feedback->current_depth);
      };

    // Result callback
    send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<auv_interfaces::action::DepthDescent>::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
      };

    // Send the goal
    depth_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void set_angle_correction(bool enable)
  {
    if (!angle_correction_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Angle correction service not available");
      return;
    }

    auto request = std::make_shared<auv_interfaces::srv::AngleCorrection::Request>();
    request->enable = enable;

    auto callback = [this, enable](rclcpp::Client<auv_interfaces::srv::AngleCorrection>::SharedFuture future) {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO(
            this->get_logger(), 
            "Successfully %s angle correction", 
            enable ? "enabled" : "disabled");
          angle_correction_enabled_ = enable;
        } else {
          RCLCPP_ERROR(
            this->get_logger(), 
            "Failed to %s angle correction", 
            enable ? "enable" : "disable");
        }
      };

    angle_correction_client_->async_send_request(request, callback);
  }

  void timer_callback()
  {
    auto twist_message = geometry_msgs::msg::Twist();
    std::string command;

    std::cout << "\nAvailable commands:\n"
              << "1. 'manual' - Switch to MANUAL mode\n"
              << "2. 'depth X' - Set target depth (X in meters)\n"
              << "3. 'angle_correction' - Toggle angle correction mode\n"
              << "4. 'move' - Input movement velocities\n"
              << "Enter command: ";

    std::getline(std::cin, command);

    if (command == "manual") {
      set_mode("MANUAL");
    } else if (command.substr(0, 5) == "depth") {
      try {
        float target_depth = std::stof(command.substr(6));
        send_depth_action(target_depth);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid depth command format. Use 'depth X'");
      }
    } else if (command == "angle_correction") {
      // Toggle angle correction
      set_angle_correction(!angle_correction_enabled_);
    } else if (command == "move") {
      std::cout << "Enter linear velocities (x y z): ";
      std::cin >> twist_message.linear.x >> twist_message.linear.y >> twist_message.linear.z;
      std::cout << "Enter angular velocities (x y z): ";
      std::cin >> twist_message.angular.x >> twist_message.angular.y >> twist_message.angular.z;
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
    }

    // Publish the Twist message
    RCLCPP_INFO(
      this->get_logger(), "Publishing Twist message: linear=(%f, %f, %f), angular=(%f, %f, %f)",
      twist_message.linear.x, twist_message.linear.y, twist_message.linear.z,
      twist_message.angular.x, twist_message.angular.y, twist_message.angular.z);
    cmd_vel_pub_->publish(twist_message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mode_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<auv_interfaces::srv::AngleCorrection>::SharedPtr angle_correction_client_;
  rclcpp_action::Client<auv_interfaces::action::DepthDescent>::SharedPtr depth_action_client_;
  std::string current_mode_;
  bool angle_correction_enabled_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AUVController>());
  rclcpp::shutdown();
  return 0;
}