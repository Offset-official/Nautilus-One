#include "rclcpp/rclcpp.hpp"

class SenseNode : public rclcpp::Node
{
public:
    SenseNode() : Node("auv_sense") {}
private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}