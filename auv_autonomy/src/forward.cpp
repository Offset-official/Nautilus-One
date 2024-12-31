#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include <memory>

using namespace std::chrono_literals;

class RCControlNode : public rclcpp::Node
{
public:
    RCControlNode()
        : Node("rc_node"),
          default_pwm(1500),
          throttle_pwm(1550)
    {
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        arm_vehicle(true);

        set_pwm_value_for_channel(5, throttle_pwm);

        reset_timer_ = this->create_wall_timer(10s, std::bind(&RCControlNode::reset_pwm, this));
    }

private:
    void arm_vehicle(bool arm)
    {
        while (!arm_client_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for arm service to be available...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        auto future = arm_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) 
                == rclcpp::FutureReturnCode::SUCCESS) {
            if (arm) {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
            } else {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed successfully");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call arm service");
        }
    }

    void set_pwm_value_for_channel(int channel, int pwm_value)
    {
        auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();

        rc_out_msg.channels = {1500, 1500, 1500, 1500, 1800, 1500, 1500, 1500};
        //rc_out_msg.set__channels( {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500});

        rc_out_msg.channels[channel - 1] = 1550;

        rc_pub_->publish(rc_out_msg);
        RCLCPP_INFO(this->get_logger(), "Set PWM value %d for channel %d", pwm_value, channel);
    }

    void reset_pwm()
    {
        set_pwm_value_for_channel(5, default_pwm);
        RCLCPP_INFO(this->get_logger(), "Reset PWM value back to default (%d)", default_pwm);
    }

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::TimerBase::SharedPtr reset_timer_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    const int default_pwm;
    const int throttle_pwm;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RCControlNode>());
    rclcpp::shutdown();
    return 0;
}
