#include <functional>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class DumbController : public rclcpp::Node {
public:
    DumbController()
        : Node("dumb_controller")
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        
        // PWM parameters
        this->declare_parameter("neutral_pwm", 1500, param_desc);
        
        this->declare_parameter("surge_low_pwm_change", 30, param_desc);
        this->declare_parameter("surge_medium_pwm_change", 40, param_desc);
        this->declare_parameter("surge_high_pwm_change", 50, param_desc);
        
        this->declare_parameter("yaw_low_pwm_change", 40, param_desc);
        this->declare_parameter("yaw_medium_pwm_change", 40, param_desc);
        this->declare_parameter("yaw_high_pwm_change", 40, param_desc);
        
        this->declare_parameter("heave_low_pwm_change", 50, param_desc);
        this->declare_parameter("heave_medium_pwm_change", 50, param_desc);
        this->declare_parameter("heave_high_pwm_change", 50, param_desc);

        // Velocity threshold parameters
        rcl_interfaces::msg::ParameterDescriptor double_param_desc;
        double_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        
        this->declare_parameter("surge_low_cut_off", 0.2, double_param_desc);
        this->declare_parameter("surge_medium_cut_off", 0.5, double_param_desc);
        this->declare_parameter("surge_high_cut_off", 0.8, double_param_desc);
        
        this->declare_parameter("yaw_low_cut_off", 0.2, double_param_desc);
        this->declare_parameter("yaw_medium_cut_off", 0.5, double_param_desc);
        this->declare_parameter("yaw_high_cut_off", 0.8, double_param_desc);
        
        this->declare_parameter("heave_low_cut_off", 0.05, double_param_desc);
        this->declare_parameter("heave_medium_cut_off", 0.3, double_param_desc);
        this->declare_parameter("heave_high_cut_off", 0.6, double_param_desc);

        this->declare_parameter("depth_p_gain", 1.0, double_param_desc);
        this->declare_parameter("depth_threshold", 0.025, double_param_desc);

        this->declare_parameter("publish_rate_ms", 50, param_desc);

        // Get PWM parameters
        neutral_pwm_ = this->get_parameter("neutral_pwm").as_int();
        surge_low_pwm_change_ = this->get_parameter("surge_low_pwm_change").as_int();
        surge_medium_pwm_change_ = this->get_parameter("surge_medium_pwm_change").as_int();
        surge_high_pwm_change_ = this->get_parameter("surge_high_pwm_change").as_int();
        
        yaw_low_pwm_change_ = this->get_parameter("yaw_low_pwm_change").as_int();
        yaw_medium_pwm_change_ = this->get_parameter("yaw_medium_pwm_change").as_int();
        yaw_high_pwm_change_ = this->get_parameter("yaw_high_pwm_change").as_int();
        
        heave_low_pwm_change_ = this->get_parameter("heave_low_pwm_change").as_int();
        heave_medium_pwm_change_ = this->get_parameter("heave_medium_pwm_change").as_int();
        heave_high_pwm_change_ = this->get_parameter("heave_high_pwm_change").as_int();

        // Get velocity threshold parameters
        surge_low_cut_off = this->get_parameter("surge_low_cut_off").as_double();
        surge_medium_cut_off = this->get_parameter("surge_medium_cut_off").as_double();
        surge_high_cut_off = this->get_parameter("surge_high_cut_off").as_double();
        
        yaw_low_cut_off = this->get_parameter("yaw_low_cut_off").as_double();
        yaw_medium_cut_off = this->get_parameter("yaw_medium_cut_off").as_double();
        yaw_high_cut_off = this->get_parameter("yaw_high_cut_off").as_double();
        
        heave_low_cut_off = this->get_parameter("heave_low_cut_off").as_double();
        heave_medium_cut_off = this->get_parameter("heave_medium_cut_off").as_double();
        heave_high_cut_off = this->get_parameter("heave_high_cut_off").as_double();

        int publish_rate = this->get_parameter("publish_rate_ms").as_int();

        // Initialize state variables
        velocity_surge = 0.0;
        velocity_yaw = 0.0;
        velocity_heave = 0.0;
        target_vel_surge = 0.0;
        target_vel_yaw = 0.0;
        target_vel_heave = 0.0;

        current_depth_ = 0.0;
        target_depth_ = 0.0;

        // Log initialization
        RCLCPP_INFO(this->get_logger(), "Initializing DumbController");

        // Set up subscriptions
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DumbController::twist_callback, this, std::placeholders::_1)); // for setting target velocities for intelligence

        current_depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/current_depth", 10,
            std::bind(&DumbController::current_depth_callback, this, std::placeholders::_1)); // for getting current depth

        target_depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_depth", 10,
            std::bind(&DumbController::target_depth_callback, this, std::placeholders::_1)); // for getting target depth

        // Set up publishers
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        
        // Set up arm client
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        
        // Arm the vehicle
        arm_vehicle(true);
        // set_mode("ALT_HOLD");
        set_mode("MANUAL");
        
        // Set up timer
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_rate), 
            std::bind(&DumbController::timer_callback, this));
    }

private:
    void timer_callback()
    {
        control_callback();
        publish_rc_override();
    }

    void current_depth_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        current_depth_ = msg->data;
    }

    void target_depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        target_depth_ = static_cast<double>(msg->data);
        depth_reached_ = false;
        RCLCPP_INFO(this->get_logger(), "New target depth: %.2f meters", target_depth_);
    }

    void set_mode(const std::string& mode)
    {
        if (!mode_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
            return;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;

        auto callback = [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            auto result = future.get();
            if (result->mode_sent) {
                RCLCPP_INFO(this->get_logger(), "Successfully set mode to: %s", mode.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set mode to: %s", mode.c_str());
            }
        };

        mode_client_->async_send_request(request, callback);
    }

    void twist_callback(const geometry_msgs::msg::Twist &msg) {
        target_vel_surge = msg.linear.y;
        target_vel_yaw = msg.angular.z;
        target_vel_heave = msg.linear.z;
    }

    void control_callback() {
        
        double depth_error = target_depth_ - current_depth_;
        double threshold = this->get_parameter("depth_threshold").as_double();
        double p_gain = this->get_parameter("depth_p_gain").as_double();

        if (!depth_reached_ && std::abs(depth_error) <= threshold) {
            RCLCPP_INFO(this->get_logger(), 
                        "Target depth %.2f reached", target_depth_);
            target_vel_heave = 0.0;
            depth_reached_ = true;
            // set_mode("ALT_HOLD");
        } else if (!depth_reached_) {
            target_vel_heave = depth_error * p_gain;
            RCLCPP_INFO(this->get_logger(), 
                      "Depth control - Current: %.2f, Target: %.2f, Error: %.2f, Command: %.2f",
                      current_depth_, target_depth_, depth_error, target_vel_heave);
        }

        if (std::abs(depth_error) > threshold){
            depth_reached_ = false;
        }

        heave_pwm_ = calculatePWM(target_vel_heave,
                    heave_high_cut_off, heave_medium_cut_off, heave_low_cut_off,
                    heave_high_pwm_change_, heave_medium_pwm_change_, heave_low_pwm_change_);
                    
        yaw_pwm_ = calculatePWM(target_vel_yaw,
                    yaw_high_cut_off, yaw_medium_cut_off, yaw_low_cut_off,
                    yaw_high_pwm_change_, yaw_medium_pwm_change_, yaw_low_pwm_change_);
                    
        surge_pwm_ = calculatePWM(target_vel_surge,
                    surge_high_cut_off, surge_medium_cut_off, surge_low_cut_off,
                    surge_high_pwm_change_, surge_medium_pwm_change_, surge_low_pwm_change_);

        RCLCPP_INFO(this->get_logger(),
                    "\n\tSurge | Yaw | Heave\nTarget_vel: [%.2f, %.2f, %.2f]\nPWM sent to thrusters: [%d, %d,%d]",
                    target_vel_surge, target_vel_yaw,target_vel_heave,
                    surge_pwm_, yaw_pwm_, heave_pwm_);

    }
    // TODO add custmized PWM for backward motion
    int calculatePWM(float target_vel,
                    const float high_cut_off, const float medium_cut_off, const float low_cut_off,
                    const int high_pwm_change, const int medium_pwm_change, const int low_pwm_change) {
        float abs_target = abs(target_vel);
        int pwm_change = 0;
        int pwm = 0;

        if (abs_target >= high_cut_off) {
            pwm_change = high_pwm_change;
        } else if (abs_target >= medium_cut_off) {
            pwm_change = medium_pwm_change;
        } else if (abs_target >= low_cut_off) {
            pwm_change = low_pwm_change;
        }

        pwm = neutral_pwm_ + (target_vel >= 0 ? pwm_change : -pwm_change);
        return pwm;
    }
    void publish_rc_override() {
        auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();
        rc_out_msg.channels = {
            static_cast<unsigned short>(neutral_pwm_),
            static_cast<unsigned short>(neutral_pwm_),
            static_cast<unsigned short>(heave_pwm_),
            static_cast<unsigned short>(yaw_pwm_),
            static_cast<unsigned short>(surge_pwm_),
            static_cast<unsigned short>(neutral_pwm_),
            static_cast<unsigned short>(neutral_pwm_),
            static_cast<unsigned short>(neutral_pwm_)
        };

        rc_pub_->publish(rc_out_msg);
        RCLCPP_DEBUG(this->get_logger(),
                     "Published RC Override: Surge: %d, Yaw: %d, Heave: %d",
                     surge_pwm_, yaw_pwm_, heave_pwm_);
    }

    void arm_vehicle(bool arm) {
        while (!arm_client_->wait_for_service(1s) && rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for arm service to be available...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        auto future = arm_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Vehicle %s successfully", arm ? "armed" : "disarmed");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call arm service");
        }
    }


    int heave_pwm_ = 1500;
    int surge_pwm_ = 1500;
    int yaw_pwm_ = 1500;
    double velocity_surge, velocity_yaw,velocity_heave;
    double target_vel_surge, target_vel_yaw,target_vel_heave;
    
    int neutral_pwm_;
    int surge_low_pwm_change_;
    int surge_medium_pwm_change_;
    int surge_high_pwm_change_;
    int yaw_low_pwm_change_;
    int yaw_medium_pwm_change_;
    int yaw_high_pwm_change_;
    int heave_low_pwm_change_;
    int heave_medium_pwm_change_;
    int heave_high_pwm_change_;

    double surge_low_cut_off;
    double surge_medium_cut_off;
    double surge_high_cut_off;
    double yaw_low_cut_off;
    double yaw_medium_cut_off;
    double yaw_high_cut_off;
    double heave_low_cut_off;
    double heave_medium_cut_off;
    double heave_high_cut_off;

    double current_depth_;
    double target_depth_;
    bool depth_reached_;

    // Time tracking
    rclcpp::Time last_time_;


    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_depth_sub_;
    
    // Publishers
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    
    // Service client
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<DumbController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
