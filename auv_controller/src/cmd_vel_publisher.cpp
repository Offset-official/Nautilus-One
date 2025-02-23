#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CmdVelPublisher : public rclcpp::Node
{
public:
  CmdVelPublisher()
  : Node("cmd_vel_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&CmdVelPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto twist_message = geometry_msgs::msg::Twist();
    std::cout << "Enter linear velocities (x y z): ";
    std::cin >> twist_message.linear.x >> twist_message.linear.y >> twist_message.linear.z;
    std::cout << "Enter angular velocities (x y z): ";
    std::cin >> twist_message.angular.x >> twist_message.angular.y >> twist_message.angular.z;
    RCLCPP_INFO(
      this->get_logger(), "Publishing Twist message: linear=(%f, %f, %f), angular=(%f, %f, %f)",
      twist_message.linear.x, twist_message.linear.y, twist_message.linear.z,
      twist_message.angular.x, twist_message.angular.y, twist_message.angular.z);
    publisher_->publish(twist_message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelPublisher>());
  rclcpp::shutdown();
  return 0;
}
