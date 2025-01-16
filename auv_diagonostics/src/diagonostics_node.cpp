#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"

class DiagonosticsNode : public rclcpp::Node{
    public:
        DiagonosticsNode() : Node("diagonostics_node"){
            state_arm_sub_ = this->create_subscription<mavros_msgs::msg::State>(
                "/mavros/state", 10, std::bind(&
            DiagonosticsNode::arm_state_callback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_arm_sub_;

        void arm_state_callback(const mavros_msgs::msg::State::SharedPtr msg){
            if(msg->armed){
                RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
            }else{
                RCLCPP_INFO(this->get_logger(), "Vehicle is disarmed");
            }
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiagonosticsNode>());
    rclcpp::shutdown();
    return 0;
}