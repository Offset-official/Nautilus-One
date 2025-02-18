#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "mavros_msgs/msg/vfr_hud.hpp"

using namespace std::chrono_literals;

class DepthControl : public rclcpp::Node
{
public:
    DepthControl() : Node("depth_controller"), depth_reached_(false)
    {
        // Publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscriber for current depth
        current_depth_sub_1 = this->create_subscription<std_msgs::msg::Float64>(
            "/current_depth", 10,
            std::bind(&DepthControl::current_depth_callback1, this, std::placeholders::_1)); // wrong topic, change later. 

        
        // Subscriber for target depth
        target_depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_depth", 10,
            std::bind(&DepthControl::target_depth_callback, this, std::placeholders::_1));

        // Timer for control loop
        timer_ = this->create_wall_timer(100ms, std::bind(&DepthControl::control_loop, this));

        // Parameters
        this->declare_parameter("depth_p_gain", 0.5);
        this->declare_parameter("depth_threshold", 0.1);
        
        RCLCPP_INFO(this->get_logger(), "Depth controller with alt hold initialized");
    }

private:
    void current_depth_callback1(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_depth_ = msg->data;
    }

    void target_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_depth_ = static_cast<double>(msg->data);
        depth_reached_ = false;  // Reset state when new target received
        RCLCPP_INFO(this->get_logger(), "New target depth: %.2f meters", target_depth_);
    }



    void control_loop()
    {
        if (depth_reached_) {
            return;  // Skip control if target depth is already reached
        }

        auto cmd_vel = geometry_msgs::msg::Twist();
        
        // Calculate depth error and control output
        double depth_error = target_depth_ - current_depth_;
        double threshold = this->get_parameter("depth_threshold").as_double();

        if (std::abs(depth_error) <= threshold) {
            
            RCLCPP_INFO(this->get_logger(), 
                        "Target depth %.2f reached. Turning OFF depth control", target_depth_);
            cmd_vel.linear.z = 0.0;
            depth_reached_ = true;
            return;

        } else {
            double p_gain = this->get_parameter("depth_p_gain").as_double();
            cmd_vel.linear.z = depth_error * p_gain;
            
            // send according to the error in limit tho. 

            RCLCPP_DEBUG(this->get_logger(), 
                      "Depth control - Current: %.2f, Target: %.2f, Error: %.2f, Command: %.2f",
                      current_depth_, target_depth_, depth_error, cmd_vel.linear.z);
        }
        vel_pub_->publish(cmd_vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_depth_sub_1;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_depth_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double current_depth_ = 0.0;
    double target_depth_ = 0.0;
    bool depth_reached_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthControl>());
    rclcpp::shutdown();
    return 0;
}