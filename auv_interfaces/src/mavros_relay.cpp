#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "auv_interfaces/msg/diagnostics.hpp"

class MavrosRelay : public rclcpp::Node
{
public:
    MavrosRelay() : Node("mavros_relay"), armed_(false)
    {
        // Subscribers to MAVROS topics
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&MavrosRelay::stateCallback, this, std::placeholders::_1));

        // Publisher for the new topic
        diagnostics_pub_ = this->create_publisher<auv_interfaces::msg::Diagnostics>("/auv_interfaces/relay", 10);

        // Timer to publish messages
        timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                         std::bind(&MavrosRelay::publishDiagnostics, this));
    }

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed_ = msg->armed;
    }

    void publishDiagnostics()
    {
        auto msg = auv_interfaces::msg::Diagnostics();
        msg.armed = armed_;

        diagnostics_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published Mediator Data: armed=%d",
                    msg.armed);
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<auv_interfaces::msg::Diagnostics>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool armed_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavrosRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
