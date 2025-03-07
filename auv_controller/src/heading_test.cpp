#include <chrono>
#include <memory>
#include <thread>

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
          target_depth_(-0.6),  // Default value
          angle_correction_(false),  // Default value
          linear_speed_(1.0),  // Default value
          movement_duration_(10.0)  // Default 10 seconds
    {
        // Declare parameters
        this->declare_parameter("target_depth", target_depth_);
        this->declare_parameter("angle_correction", angle_correction_);
        this->declare_parameter("linear_speed", linear_speed_);
        this->declare_parameter("movement_duration", movement_duration_);
        
        // Get parameters
        target_depth_ = this->get_parameter("target_depth").as_double();
        angle_correction_ = this->get_parameter("angle_correction").as_bool();
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
            
        // Start the execution sequence in a separate thread to avoid blocking the main thread
        std::thread([this]() { this->execute_sequence(); }).detach();
    }

private:
    void execute_sequence()
    {
        RCLCPP_INFO(this->get_logger(), "Starting execution sequence");
        RCLCPP_INFO(this->get_logger(), "Target depth: %f", target_depth_);
        RCLCPP_INFO(this->get_logger(), "Angle correction: %s", angle_correction_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Linear speed: %f", linear_speed_);
        RCLCPP_INFO(this->get_logger(), "Movement duration: %f seconds", movement_duration_);
        
        // Step 1: Enable angle correction
        if (angle_correction_) {
            RCLCPP_INFO(this->get_logger(), "Step 1: Enabling angle correction");
            set_angle_correction(true);
            
            // Wait for angle correction to be enabled
            int attempts = 0;
            while (!angle_correction_ && attempts < 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                rclcpp::spin_some(this->get_node_base_interface());
                attempts++;
            }
        }
        
        // Step 2: Wait for 10 seconds
        RCLCPP_INFO(this->get_logger(), "Step 2: Waiting for 10 seconds");
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // Step 3: Send depth action
        RCLCPP_INFO(this->get_logger(), "Step 3: Starting descent to target depth: %f", target_depth_);
        bool depth_action_completed = false;
        send_depth_action(target_depth_, [this, &depth_action_completed]() {
            depth_action_completed = true;
        });
        
        // Wait for depth action to complete
        while (!depth_action_completed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(this->get_node_base_interface());
        }
        
        // Step 4: Wait for 5 seconds after depth descent
        RCLCPP_INFO(this->get_logger(), "Step 4: Waiting for 5 seconds after depth descent");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // Step 5: Move forward for specified duration
        RCLCPP_INFO(this->get_logger(), "Step 5: Moving forward with speed %f for %f seconds", 
                    linear_speed_, movement_duration_);
        perform_forward_movement();
        
        // Step 6: Wait for 5 seconds after forward motion
        RCLCPP_INFO(this->get_logger(), "Step 6: Waiting for 5 seconds after forward motion");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        // Step 7: Return to surface
        RCLCPP_INFO(this->get_logger(), "Step 7: Returning to surface (depth 0.025)");
        depth_action_completed = false;
        send_depth_action(0.025, [this, &depth_action_completed]() {
            depth_action_completed = true;
        });
        
        // Wait for return to surface to complete
        while (!depth_action_completed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(this->get_node_base_interface());
        }
        
        // Step 8: Final log message
        RCLCPP_INFO(this->get_logger(), "Sequence completed successfully!");
    }

    void send_depth_action(float target_depth, std::function<void()> completion_callback)
    {
        if (!depth_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Depth descent action server not available after waiting 5 seconds");
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
            [this, completion_callback](const rclcpp_action::ClientGoalHandle<auv_interfaces::action::DepthDescent>::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Depth action succeeded with result: %f", result.result->final_depth);
                if (completion_callback) {
                    completion_callback();
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Depth action did not succeed, result code: %d", 
                             static_cast<int>(result.code));
                if (completion_callback) {
                    completion_callback();  // Still call callback to prevent deadlock
                }
            }
        };

        // Send the goal
        depth_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void perform_forward_movement()
    {
        // Start moving forward
        auto twist_message = geometry_msgs::msg::Twist();
        twist_message.linear.y = linear_speed_;
        
        // Time tracking
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::duration<double>(movement_duration_);
        
        // Move forward for the specified duration
        while (std::chrono::steady_clock::now() < end_time) {
            cmd_vel_pub_->publish(twist_message);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            rclcpp::spin_some(this->get_node_base_interface());
        }
        
        // Stop movement
        auto stop_message = geometry_msgs::msg::Twist();
        stop_message.linear.y = 0.0;
        cmd_vel_pub_->publish(stop_message);
        
        // Publish stop command multiple times to ensure it's received
        for (int i = 0; i < 5; i++) {
            cmd_vel_pub_->publish(stop_message);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        RCLCPP_INFO(this->get_logger(), "Forward movement completed");
    }

    void set_angle_correction(bool enable)
    {
        if (!angle_correction_client_->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Angle correction service not available after waiting 5 seconds");
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
                angle_correction_ = enable;
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
    
    // Parameters
    double target_depth_;
    bool angle_correction_;
    double linear_speed_;
    double movement_duration_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadingTest>());
    rclcpp::shutdown();
    
    return 0;
}