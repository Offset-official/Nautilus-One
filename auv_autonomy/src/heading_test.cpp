#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "auv_interfaces/action/depth_descent.hpp"
#include "auv_interfaces/srv/angle_correction.hpp"
using namespace std::chrono_literals;

class HeadingTest : public rclcpp::Node
{
public:
    HeadingTest()
        : Node("heading_test"),
          angle_correction_enabled_(false),
          target_depth_(-0.6),  // Default value
          enable_angle_correction_(true),  // Default value
          linear_speed_(1.0),  // Default value
          movement_duration_(10.0)  // Default 10 seconds
    {
        // Declare parameters
        this->declare_parameter("target_depth", target_depth_);
        this->declare_parameter("enable_angle_correction", enable_angle_correction_);
        this->declare_parameter("linear_speed", linear_speed_);
        this->declare_parameter("movement_duration", movement_duration_);
        
        // Get parameters
        target_depth_ = this->get_parameter("target_depth").as_double();
        enable_angle_correction_ = this->get_parameter("enable_angle_correction").as_bool();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        movement_duration_ = this->get_parameter("movement_duration").as_double();
        
        // Initialize publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Initialize action client
        depth_action_client_ = rclcpp_action::create_client<auv_interfaces::action::DepthDescent>(
            this, "depth_descent");
            
        // Initialize service client
        angle_correction_client_ = this->create_client<auv_interfaces::srv::AngleCorrection>(
            "angle_correction");
            
        // Start the execution sequence
        this->timer_ = this->create_wall_timer(
            100ms, std::bind(&HeadingTest::execute_sequence, this));
    }

private:
    void execute_sequence()
    {
        // Only execute once
        this->timer_->cancel();
        
        RCLCPP_INFO(this->get_logger(), "Starting execution sequence");
        RCLCPP_INFO(this->get_logger(), "Target depth: %f", target_depth_);
        RCLCPP_INFO(this->get_logger(), "Angle correction: %s", enable_angle_correction_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Linear speed: %f", linear_speed_);
        RCLCPP_INFO(this->get_logger(), "Movement duration: %f seconds", movement_duration_);
        
        // Send depth action
        send_depth_action(target_depth_);
        if (enable_angle_correction_) {
            set_angle_correction(true);
        }
                
        start_forward_movement();
    }

    void send_depth_action(float target_depth)
    {
        if (!depth_action_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Depth descent action server not available");
            return;
        }

        auto goal_msg = auv_interfaces::action::DepthDescent::Goal();
        goal_msg.target_depth = target_depth;

        RCLCPP_INFO(this->get_logger(), "Sending depth descent goal: %f", target_depth);

        auto send_goal_options = rclcpp_action::Client<auv_interfaces::action::DepthDescent>::SendGoalOptions();

        // Feedback callback
        send_goal_options.feedback_callback =
            [this](
                rclcpp_action::ClientGoalHandle<auv_interfaces::action::DepthDescent>::SharedPtr,
                const std::shared_ptr<const auv_interfaces::action::DepthDescent::Feedback> feedback)
        {
            RCLCPP_INFO(
                this->get_logger(), "Depth descent feedback - Current depth: %f", feedback->current_depth);
        };

        // Goal response callback
        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<auv_interfaces::action::DepthDescent>::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            }
        };

        // Result callback
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<auv_interfaces::action::DepthDescent>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Depth action succeeded. Starting next phase.");
                

            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Depth action did not succeed");
            }
        };

        // Send the goal
        auto req_ = depth_action_client_->async_send_goal(goal_msg, send_goal_options);
        req_.wait();
    }

    void start_forward_movement()
    {
        // Start moving forward
        auto twist_message = geometry_msgs::msg::Twist();
        twist_message.linear.y = linear_speed_;
        RCLCPP_INFO(this->get_logger(), "Moving forward with speed: %f for %f seconds", 
                    linear_speed_, movement_duration_);
        
        // Publish velocity command continuously
        move_timer_ = this->create_wall_timer(
            50ms,
            [this, twist_message]() {
                cmd_vel_pub_->publish(twist_message);
            });
        
        // Create a timer to stop movement after the specified duration
        stop_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(movement_duration_),
            [this]() {
                // Stop the movement
                auto stop_message = geometry_msgs::msg::Twist();
                stop_message.linear.y = 0.0;
                RCLCPP_INFO(this->get_logger(), "Stop moving forward after %f seconds.", movement_duration_);
                cmd_vel_pub_->publish(stop_message);
                
                // Cancel the move timer
                move_timer_->cancel();
                
                // Cancel this timer
                stop_timer_->cancel();
            });
    }

    void set_angle_correction(bool enable)
    {
        if (!angle_correction_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Angle correction service not available");
            return;
        }

        auto request = std::make_shared<auv_interfaces::srv::AngleCorrection::Request>();
        request->enable = enable;

        auto callback = [this, enable](rclcpp::Client<auv_interfaces::srv::AngleCorrection>::SharedFuture future)
        {
            auto result = future.get();
            if (result->success)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Successfully %s angle correction",
                    enable ? "enabled" : "disabled");
                angle_correction_enabled_ = enable;
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Failed to %s angle correction",
                    enable ? "enable" : "disable");
            }
        };

        angle_correction_client_->async_send_request(request, callback);
    }

    // Publishers, clients, and member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Client<auv_interfaces::action::DepthDescent>::SharedPtr depth_action_client_;
    rclcpp::Client<auv_interfaces::srv::AngleCorrection>::SharedPtr angle_correction_client_;
    bool angle_correction_enabled_;
    
    // Parameters
    double target_depth_;
    bool enable_angle_correction_;
    double linear_speed_;
    double movement_duration_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr move_timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadingTest>());
    rclcpp::shutdown();
    
    return 0;
}