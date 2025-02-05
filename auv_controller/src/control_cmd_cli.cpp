#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class DepthHold : public rclcpp::Node
{
public:
    DepthHold()
    : Node("depth_hold"), 
      current_mode_(""),
      target_depth_(0.0),
      current_depth_(0.0),
      depth_control_active_(false)
    {
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscribers
        mode_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10,
            std::bind(&DepthHold::mode_callback, this, std::placeholders::_1));
            
        depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "mavros/global_position/rel_alt", 10,
            std::bind(&DepthHold::depth_callback, this, std::placeholders::_1));

        depth_pub_ = this->create_publisher<std_msgs::msg::Float32>("depth", 10);

        // Service clients
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

        // Timer for main control loop
        timer_ = this->create_wall_timer(500ms, std::bind(&DepthHold::timer_callback, this));

        // Parameters
        this->declare_parameter("depth_p_gain", 0.5);
        this->declare_parameter("depth_threshold", 0.1);
        
        RCLCPP_INFO(this->get_logger(), "DepthHold initialized");
    }

private:
    void mode_callback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_mode_ = msg->mode;
        RCLCPP_INFO(this->get_logger(), "Current mode: %s", current_mode_.c_str());
    }

    void depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_depth_ = msg->data;
    }

    void set_mode(const std::string& mode)
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

    void timer_callback()
    {
        auto twist_message = geometry_msgs::msg::Twist();
        std::string command;
        
        std::cout << "\nAvailable commands:\n"
                  << "1. 'manual' - Switch to MANUAL mode\n"
                  << "2. 'stabilize' - Switch to STABILIZE mode\n"
                  << "3. 'depth_hold' - Switch to DEPTH_HOLD mode\n"
                  << "4. 'depth X' - Set target depth (X in meters)\n"
                  << "5. 'move' - Input movement velocities\n"
                  << "Enter command: ";
        
        std::getline(std::cin, command);

        if (command == "manual") {
            set_mode("MANUAL");
            depth_control_active_ = false;
        }
        else if (command == "stabilize") {
            set_mode("STABILIZE");
            depth_control_active_ = false;
        }
        else if (command == "depth_hold") {
            set_mode("ALT_HOLD");
            depth_control_active_ = false;
        }
        else if (command.substr(0, 5) == "depth") {
            try {
                target_depth_ = std::stof(command.substr(6));
                depth_control_active_ = true;
                auto message = std_msgs::msg::Float32();
                message.data = target_depth_;
                depth_pub_->publish(message);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Invalid depth command format. Use 'depth X'");
            }
        }
        else if (command == "move") {
            std::cout << "Enter linear velocities (x y z): ";
            std::cin >> twist_message.linear.x >> twist_message.linear.y >> twist_message.linear.z;
            std::cout << "Enter angular velocities (x y z): ";
            std::cin >> twist_message.angular.x >> twist_message.angular.y >> twist_message.angular.z;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Clear input buffer
        }

        // Apply depth control if active
        if (depth_control_active_) {
            double depth_error = target_depth_ - current_depth_;
            double depth_p_gain = this->get_parameter("depth_p_gain").as_double();
            double depth_threshold = this->get_parameter("depth_threshold").as_double();

            if (std::abs(depth_error) > depth_threshold) {
                twist_message.linear.z = depth_error * depth_p_gain;
                RCLCPP_INFO(this->get_logger(), 
                           "Depth control active - Current: %.2f, Target: %.2f, Error: %.2f, Command: %.2f",
                           current_depth_, target_depth_, depth_error, twist_message.linear.z);
            }
        }

        // Publish the Twist message
        RCLCPP_INFO(this->get_logger(), 
                    "Publishing Twist message: linear=(%f, %f, %f), angular=(%f, %f, %f)",
                    twist_message.linear.x, twist_message.linear.y, twist_message.linear.z,
                    twist_message.angular.x, twist_message.angular.y, twist_message.angular.z);
        cmd_vel_pub_->publish(twist_message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depth_pub_;
    std::string current_mode_;
    double target_depth_;
    double current_depth_;
    bool depth_control_active_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthHold>());
    rclcpp::shutdown();
    return 0;
}