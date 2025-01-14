#include <functional>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"

using namespace std::chrono_literals;

class KalmanFilter
{
public:
    KalmanFilter(double process_noise, double measurement_noise, double estimated_error, double initial_value)
        : process_noise_(process_noise),
          measurement_noise_(measurement_noise),
          estimated_error_(estimated_error),
          value_(initial_value),
          velocity_(0.0) {}

    // Overloaded update for acceleration integration
    double update(double measurement, double dt)
    {
        if (std::abs(measurement) < 0.02) { 
            measurement = 0.0;
        }

        estimated_error_ += process_noise_;
        
        velocity_ += measurement * dt;
        

        return velocity_;
    }

    // Original update method for angular velocity
    double update(double measurement)
    {
        // Apply threshold for near-zero measurements
        if (std::abs(measurement) < 0.005) { // Adjust threshold based on your IMU's noise characteristics
            measurement = 0.0;
        }

        // Prediction update
        estimated_error_ += process_noise_;

        // Measurement update
        double kalman_gain = estimated_error_ / (estimated_error_ + measurement_noise_);
        value_ += kalman_gain * (measurement - value_);
        estimated_error_ *= (1 - kalman_gain);

        return value_;
    }


private:
    double process_noise_;
    double measurement_noise_;
    double estimated_error_;
    double value_;
    double velocity_;
};

class BaseController : public rclcpp::Node
{
public:
    BaseController(bool use_kalman, bool use_pid)
        : Node("base_controller"),
          neutral_pwm(1500),
          pwm_range(50),
          kp_(0.6),      
          ki_(0.05),     
          kd_(0.15),
          use_kalman_(use_kalman),
          use_pid_(use_pid),
          velocity_surge(0.0),
          velocity_yaw(0.0),
          velocity_heave(0.0),
          target_vel_surge(0.0),
          target_vel_yaw(0.0),
          target_vel_heave(0.0),
          last_time_(this->now()),
          kalman_surge(0.005, 0.2, 1.0, 0.0),  
          kalman_yaw(0.005, 0.2, 1.0, 0.0),
          kalman_heave(0.005, 0.2, 1.0, 0.0)
    {

        RCLCPP_INFO(this->get_logger(), "Initializing BaseController with:");
        RCLCPP_INFO(this->get_logger(), "Kalman Filter: %s", use_kalman_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "PID Controller: %s", use_pid_ ? "ON" : "OFF");

        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&BaseController::twist_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", qos_profile,
            std::bind(&BaseController::imu_callback, this, std::placeholders::_1));

        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("applied_vel", 10);
        imu_alt_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_vel", 10);
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        arm_vehicle(true);

        publish_timer_ = this->create_wall_timer(
            100ms, std::bind(&BaseController::publish_rc_override, this));
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist &msg)
    {
        target_vel_surge = msg.linear.y;
        target_vel_heave = msg.linear.z;
        target_vel_yaw = msg.angular.z; 

        RCLCPP_INFO(this->get_logger(),
                    "Set Target Velocities: Heave: %.2f, Surge: %.2f, Yaw: %.2f",
                    target_vel_heave, target_vel_surge, target_vel_yaw);
    }

    void imu_callback(const sensor_msgs::msg::Imu &msg)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        double lin_acc_heave = msg.linear_acceleration.z - 9.816; // Subtract gravity
        double lin_acc_surge = -(msg.linear_acceleration.y - 0.10787)*1.4; // Subtract bias
        double ang_vel_yaw = -msg.angular_velocity.z - 0.00042; // Subtract bias 

        double filtered_acc_surge, velocity_yaw, filtered_acc_heave;
        if (use_kalman_) {
            filtered_acc_surge = kalman_surge.update(lin_acc_surge);
            velocity_yaw = kalman_yaw.update(ang_vel_yaw);
            filtered_acc_heave = kalman_heave.update(lin_acc_heave);
        } else {
            filtered_acc_surge = lin_acc_surge;
            velocity_yaw = ang_vel_yaw;
            filtered_acc_heave = lin_acc_heave;
        }

        
        
        RCLCPP_INFO(this->get_logger(),
                    "Surge acc: %.2f->%.2f, Yaw vel %.2f->%.2f, Heave acc %.2f->%.2f",
                    lin_acc_surge,filtered_acc_surge, ang_vel_yaw,velocity_yaw, lin_acc_heave,filtered_acc_heave);

        // integrate to get velocity from acceleration
        velocity_surge += filtered_acc_surge * dt;
        velocity_heave += filtered_acc_heave * dt;
        auto imu_ref_msg = geometry_msgs::msg::Vector3();
        imu_ref_msg.x = velocity_surge;  // Linear acceleration y
        imu_ref_msg.y = velocity_heave; // Linear acceleration z
        imu_ref_msg.z = ang_vel_yaw;   // Angular velocity y
        imu_alt_publisher_->publish(imu_ref_msg);

        double control_surge, control_yaw, control_heave;
        if (use_pid_) {
            // Use PID control
            control_surge = compute_pid(target_vel_surge, velocity_surge, error_surge, integral_surge, previous_error_surge, dt);
            control_yaw = compute_pid(target_vel_yaw, velocity_yaw, error_yaw, integral_yaw, previous_error_yaw, dt);
            control_heave = compute_pid(target_vel_heave, velocity_heave, error_heave, integral_heave, previous_error_heave, dt);
        } else {
            control_surge = target_vel_surge - velocity_surge;
            control_yaw = target_vel_yaw - velocity_yaw;
            control_heave = target_vel_heave - velocity_heave;
        }
        // Map PID output to PWM
        surge_pwm_ = map_to_pwm(control_surge);
        yaw_pwm_ = map_to_pwm(control_yaw);
        heave_pwm_ = map_to_pwm(control_heave);
        // Publish velocity for debugging
        auto velocity_msg = geometry_msgs::msg::Vector3();
        velocity_msg.x = velocity_surge;
        velocity_msg.y = velocity_yaw;
        velocity_msg.z = velocity_heave;
        velocity_publisher_->publish(velocity_msg);

        RCLCPP_INFO(this->get_logger(),
                    "\nTarget_vel: [%.2f, %.2f, %.2f]\nMeasured_vel: [%.2f, %.2f, %.2f]\nControl: [%.2f, %.2f, %.2f]\nPWM sent to thrusters: [%d, %d, %d]",
                    target_vel_surge, target_vel_yaw, target_vel_heave,
                    velocity_surge, velocity_yaw, velocity_heave,
                    control_surge, control_yaw, control_heave,
                    surge_pwm_, yaw_pwm_, heave_pwm_);
 
        last_time_ = current_time;
    }

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
                     "Published RC Override: Heave: %d, Surge: %d, Yaw: %d",
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

    double compute_pid(double target, double measured, double &error, double &integral, double &previous_error, double dt)
    {
        error = target - measured;

        // Increased threshold for integral reset
        if (std::abs(target) < 0.02 || std::abs(error) < 0.02) {  // Increased from 0.01
            integral = 0.0;
        } else {
            integral += error * dt;
        }

        // More aggressive integral clamping
        integral = std::clamp(integral, -5.0, 5.0);  // Reduced from ±10.0

        double derivative = (error - previous_error) / dt;
        previous_error = error;

        double output = kp_ * error + ki_ * integral + kd_ * derivative;

        // Increased deadband
        if (std::abs(output) < 0.15) {  // Increased from 0.1
            output = 0.0;
        }
        
        return output;
    }


    int map_to_pwm(double input)
    {
        input = std::max(-1.0, std::min(1.0, input));
        return static_cast<int>(neutral_pwm + input * pwm_range);
    }

    int heave_pwm_ = 1500;
    int surge_pwm_ = 1500;
    int yaw_pwm_ = 1500;

    const int neutral_pwm;
    const int pwm_range;

    // PID parameters
    double kp_, ki_, kd_;

    bool use_kalman_;
    bool use_pid_;

    double velocity_surge, velocity_yaw, velocity_heave;
    double target_vel_surge, target_vel_yaw, target_vel_heave;

    double error_surge, error_yaw, error_heave;
    double integral_surge, integral_yaw, integral_heave;
    double previous_error_surge, previous_error_yaw, previous_error_heave;

    rclcpp::Time last_time_;
    KalmanFilter kalman_surge, kalman_yaw, kalman_heave;
    

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_alt_publisher_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("base_controller_params");
    bool use_kalman = node->declare_parameter("use_kalman", true);
    bool use_pid = node->declare_parameter("use_pid", true);
    
    auto controller = std::make_shared<BaseController>(use_kalman, use_pid);
    rclcpp::spin(std::make_shared<BaseController>(use_kalman, use_pid));
    rclcpp::shutdown();
    return 0;
}