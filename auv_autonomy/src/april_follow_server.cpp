#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>
#include <memory>
#include <thread>

using namespace std::placeholders;
class AprilFollowActionServer : public rclcpp::Node {
public:
  explicit AprilFollowActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("april_follow_action_server", options) {}

private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilFollowActionServer>());
  rclcpp::shutdown();
  return 0;
}
