#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class CmdVelSubscriber : public rclcpp::Node
{
public:
  CmdVelSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&CmdVelSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard while listening to topic 'cmd_vel' : linear=(%f, %f, %f), angular=(%f, %f, %f)",
          msg.linear.x, msg.linear.y, msg.linear.z,
          msg.angular.x, msg.angular.y, msg.angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}
