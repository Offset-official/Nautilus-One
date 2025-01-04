#include <functional>
#include <chrono>
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TwistToThrust : public rclcpp::Node
{
public:
    TwistToThrust()
        : Node("twist_to_thrust"),
          neutral_pwm(1500),
          pwm_range(50) // PWM range for [-1.0, 1.0]
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TwistToThrust::topic_callback, this, std::placeholders::_1));
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // Arm the vehicle
        arm_vehicle(true);

        // Publish RC overrides periodically
        publish_timer_ = this->create_wall_timer(
            100ms, std::bind(&TwistToThrust::publish_rc_override, this));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist &msg)
    {
        heave_pwm_ = map_to_pwm(msg.linear.z);
        surge_pwm = map_to_pwm(msg.linear.x);
        yaw_pwm_ = map_to_pwm(msg.angular.z);

        RCLCPP_INFO(this->get_logger(),
                    "Received cmd_vel: Heave: %.2f, Forward: %.2f, Yaw: %.2f -> PWM: %d, %d, %d",
                    msg.linear.z, msg.linear.x, msg.angular.z,
                    heave_pwm_, surge_pwm, yaw_pwm_);
    }

    void publish_rc_override()
    {
        auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();
        rc_out_msg.channels = {
            neutral_pwm, 
            neutral_pwm, 
            heave_pwm_,  // Channel 3: Heave (up/down)
            yaw_pwm_,    // Channel 4: Yaw (rotation)
            surge_pwm,   // Channel 5: Surge (forward/backward)
            neutral_pwm, 
            neutral_pwm, 
            neutral_pwm  
        };

        rc_pub_->publish(rc_out_msg);
        RCLCPP_DEBUG(this->get_logger(),
                     "Published RC Override: Heave: %d, Forward: %d, Yaw: %d",
                     heave_pwm_, surge_pwm, yaw_pwm_);
    }

    void arm_vehicle(bool arm)
    {
        while (!arm_client_->wait_for_service(1s) && rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for arm service to be available...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        auto future = arm_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (arm)
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed successfully");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call arm service");
        }
    }

    int map_to_pwm(double input)
    {
        input = std::max(-1.0, std::min(1.0, input));
        return static_cast<int>(neutral_pwm + input * pwm_range);
    }

    int heave_pwm_ = 1500;
    int surge_pwm = 1500;
    int yaw_pwm_ = 1500;

    const int neutral_pwm;
    const int pwm_range;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToThrust>());
    rclcpp::shutdown();
    return 0;
}
