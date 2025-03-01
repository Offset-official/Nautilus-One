#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "auv_interfaces/srv/dropper_trigger.hpp"  

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Dropper : public rclcpp::Node
{
public:
  Dropper()
  : Node("dropper"), neutral_pwm_(1500)  // Initialize neutral_pwm_ in the initializer list
  {
    // Parameters
    this->declare_parameter("neutral_pwm", 1500);  // Add this parameter
    this->declare_parameter("pulse_pwm", 1540);
    this->declare_parameter("pulse_duration_ms", 200);  
    
    neutral_pwm_ = this->get_parameter("neutral_pwm").as_int();  // Get from parameter
    pulse_pwm_ = this->get_parameter("pulse_pwm").as_int();
    pulse_duration_ms_ = this->get_parameter("pulse_duration_ms").as_int();
    
    // Create RC publisher
    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
      "/mavros/rc/override", 10);
    
    // Publish neutral values on startup to ensure we're in a safe state
    publish_neutral();
    
    // Create service
    dropper_service_ = this->create_service<auv_interfaces::srv::DropperTrigger>(
      "dropper_trigger", std::bind(&Dropper::handle_dropper_trigger, this, _1, _2));
      
    RCLCPP_INFO(this->get_logger(), "Dropper node initialized");
    RCLCPP_INFO(this->get_logger(), 
                "Neutral PWM: %d, Pulse PWM: %d, Pulse Duration: %d ms", 
                neutral_pwm_, pulse_pwm_, pulse_duration_ms_);
  }

private:
  void handle_dropper_trigger(
    const std::shared_ptr<auv_interfaces::srv::DropperTrigger::Request> request,
    std::shared_ptr<auv_interfaces::srv::DropperTrigger::Response> response)
  {
    if (!request->enable) {
      publish_neutral();
      response->success = true;
      response->message = "Sent neutral PWM to all channels";
      return;
    }
    
    RCLCPP_INFO(
      this->get_logger(), 
      "Sending pulse to channel 6 with PWM %d for %d ms",
      pulse_pwm_, pulse_duration_ms_);
    
    // Send pulse
    publish_pulse();
    
    // Wait for the fixed duration
    std::this_thread::sleep_for(std::chrono::milliseconds(pulse_duration_ms_));
    
    // Return to neutral
    publish_neutral();
    
    response->success = true;
    response->message = "Pulse completed successfully";
    
    RCLCPP_INFO(this->get_logger(), "Pulse completed, returned to neutral");
  }

  void publish_pulse()
  {
    auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();
    rc_out_msg.channels = {
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_),
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_),
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(pulse_pwm_),  // 6th channel
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_)};

    rc_pub_->publish(rc_out_msg);
    RCLCPP_DEBUG(
      this->get_logger(), "Published RC Override with pulse on channel 6: %d", pulse_pwm_);
  }

  void publish_neutral()
  {
    auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();
    rc_out_msg.channels = {
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_),
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_),
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_),
      static_cast<unsigned short>(neutral_pwm_), static_cast<unsigned short>(neutral_pwm_)};

    rc_pub_->publish(rc_out_msg);
    RCLCPP_DEBUG(this->get_logger(), "Published neutral RC Override on all channels");
  }

  int neutral_pwm_;
  int pulse_pwm_;
  int pulse_duration_ms_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Service<auv_interfaces::srv::DropperTrigger>::SharedPtr dropper_service_;  // Renamed to match functionality
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Dropper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}