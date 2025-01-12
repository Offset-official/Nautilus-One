#include <functional>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp" // For publishing velocities
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

using namespace std::chrono_literals;

class BaseController : public rclcpp::Node
{
public:
    BaseController()
        : Node("base_controller"),
          neutral_pwm(1500),
          pwm_range(50),
          velocity_x_(0.0),
          velocity_y_(0.0),
          velocity_z_(0.0),
          last_time_(this->now())
    {
        // Subscribers
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&BaseController::twist_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", qos_profile,
            std::bind(&BaseController::imu_callback, this, std::placeholders::_1));

        // Publishers
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("velocity", 10);

        // Client
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // Arm the vehicle
        arm_vehicle(true);

        // Timer for RC override
        publish_timer_ = this->create_wall_timer(
            100ms, std::bind(&BaseController::publish_rc_override, this));
    }

private:
    // Twist callback for cmd_vel
    void twist_callback(const geometry_msgs::msg::Twist &msg)
    {
        heave_pwm_ = map_to_pwm(msg.linear.z);
        surge_pwm_ = map_to_pwm(msg.linear.x);
        yaw_pwm_ = map_to_pwm(msg.angular.z);

        RCLCPP_INFO(this->get_logger(),
                    "Received cmd_vel: Heave: %.2f, Forward: %.2f, Yaw: %.2f -> PWM: %d, %d, %d",
                    msg.linear.z, msg.linear.x, msg.angular.z,
                    heave_pwm_, surge_pwm_, yaw_pwm_);
    }

    // IMU callback for velocity calculation
    void imu_callback(const sensor_msgs::msg::Imu &msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        velocity_x_ += msg.linear_acceleration.x * dt;
        velocity_y_ += msg.linear_acceleration.y * dt;
        velocity_z_ += msg.linear_acceleration.z * dt;

        auto velocity_msg = geometry_msgs::msg::Vector3();
        velocity_msg.x = velocity_x_;
        velocity_msg.y = velocity_y_;
        velocity_msg.z = velocity_z_;
        velocity_publisher_->publish(velocity_msg);

        last_time_ = current_time;

        RCLCPP_INFO(this->get_logger(),
                    "IMU Velocity: x=%.2f, y=%.2f, z=%.2f",
                    velocity_x_, velocity_y_, velocity_z_);
    }

    // Publish RC overrides
    void publish_rc_override()
    {
        auto rc_out_msg = mavros_msgs::msg::OverrideRCIn();
        rc_out_msg.channels = {
            static_cast<unsigned short>(neutral_pwm),
            static_cast<unsigned short>(neutral_pwm),
            static_cast<unsigned short>(heave_pwm_),  // Channel 3: Heave (up/down)
            static_cast<unsigned short>(yaw_pwm_),    // Channel 4: Yaw (rotation)
            static_cast<unsigned short>(surge_pwm_),  // Channel 5: Surge (forward/backward)
            static_cast<unsigned short>(neutral_pwm),
            static_cast<unsigned short>(neutral_pwm),
            static_cast<unsigned short>(neutral_pwm)
        };

        rc_pub_->publish(rc_out_msg);
        RCLCPP_DEBUG(this->get_logger(),
                     "Published RC Override: Heave: %d, Forward: %d, Yaw: %d",
                     heave_pwm_, surge_pwm_, yaw_pwm_);
    }

    void arm_vehicle(bool arm)
    {
        while (!arm_client_->wait_for_service(1s) && rclcpp::ok())
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for arm service to be available...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm;

        auto future = arm_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (arm)
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed successfully");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call arm service");
        }
    }

    // Maps input velocity to a PWM value
    int map_to_pwm(double input)
    {
        input = std::max(-1.0, std::min(1.0, input));
        return static_cast<int>(neutral_pwm + input * pwm_range);
    }
    
    // RC override values
    int heave_pwm_ = 1500;
    int surge_pwm_ = 1500;
    int yaw_pwm_ = 1500;

    // Constants
    const int neutral_pwm;
    const int pwm_range;

    // IMU variables
    double velocity_x_;
    double velocity_y_;
    double velocity_z_;

    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BaseController>());
    rclcpp::shutdown();
    return 0;
}
