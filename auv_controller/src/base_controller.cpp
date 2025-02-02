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

class KalmanFilter {
public:
    // Constructor to initialize the Kalman filter with given parameters
    KalmanFilter(double process_noise, double measurement_noise, double estimated_error, double initial_value)
        : process_noise_(process_noise),
          measurement_noise_(measurement_noise),
          estimated_error_(estimated_error),
          value_(initial_value) {}

    double update(double measurement) {
        if (std::abs(measurement) < measurement_threshold_) {
            measurement = 0.0;
        }
        // Update the estimated error with process noise
        estimated_error_ += process_noise_;
        // Calculate the Kalman gain
        double kalman_gain = estimated_error_ / (estimated_error_ + measurement_noise_);
        // Update the value with the new measurement
        value_ += kalman_gain * (measurement - value_);
        // Update the estimated error
        estimated_error_ *= (1 - kalman_gain);
        return value_;
    }

    void set_thresholds(double measurement_threshold, double angular_threshold) {
        measurement_threshold_ = measurement_threshold;
        angular_threshold_ = angular_threshold;
    }

private:
    double process_noise_;          
    double measurement_noise_;      
    double estimated_error_;       
    double value_;                  // Current value
    double measurement_threshold_ = 0.02;  
    double angular_threshold_ = 0.005;   
};

class BaseController : public rclcpp::Node {
public:
    BaseController()
        : Node("base_controller"),
          kalman_surge(0.01, 0.5, 1.0, 0.0),
          kalman_yaw(0.01, 0.5, 1.0, 0.0),
          kalman_heave(0.01, 0.5, 1.0, 0.0)
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
        this->declare_parameter("use_kalman", 1, param_desc);
        this->declare_parameter("use_pid", 1, param_desc);
        
        // Integer parameters
        this->declare_parameter("neutral_pwm", 1500, param_desc);
        this->declare_parameter("pwm_range", 50, param_desc);
        this->declare_parameter("pwm_deadband", 15, param_desc);
        this->declare_parameter("publish_rate_ms", 100, param_desc);

        // Double parameters
        param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        this->declare_parameter("pid_kp", 1.0, param_desc);
        this->declare_parameter("pid_ki", 0.05, param_desc);
        this->declare_parameter("pid_kd", 0.5, param_desc);
        this->declare_parameter("integral_clamp", 5.0, param_desc);
        this->declare_parameter("imu_surge_bias", 0.10787, param_desc);
        this->declare_parameter("imu_gravity", 9.81, param_desc);
        this->declare_parameter("kalman_process_noise", 0.01, param_desc);
        this->declare_parameter("kalman_measurement_noise", 0.5, param_desc);
        this->declare_parameter("kalman_estimated_error", 1.0, param_desc);
        this->declare_parameter("kalman_initial_value", 0.0, param_desc);
        this->declare_parameter("measurement_threshold", 0.02, param_desc);
        this->declare_parameter("angular_threshold", 0.005, param_desc);

        // Get parameters
        use_kalman_ = this->get_parameter("use_kalman").as_int() == 1;
        use_pid_ = this->get_parameter("use_pid").as_int() == 1;
        neutral_pwm_ = this->get_parameter("neutral_pwm").as_int();
        pwm_range_ = this->get_parameter("pwm_range").as_int();
        kp_ = this->get_parameter("pid_kp").as_double();
        ki_ = this->get_parameter("pid_ki").as_double();
        kd_ = this->get_parameter("pid_kd").as_double();
        imu_surge_bias_ = this->get_parameter("imu_surge_bias").as_double();
        imu_gravity_ = this->get_parameter("imu_gravity").as_double();
        pwm_deadband_ = this->get_parameter("pwm_deadband").as_int();
        integral_clamp_ = this->get_parameter("integral_clamp").as_double();
        measurement_threshold_ = this->get_parameter("measurement_threshold").as_double();
        angular_threshold_ = this->get_parameter("angular_threshold").as_double();
        int publish_rate = this->get_parameter("publish_rate_ms").as_int();

        // Get Kalman filter parameters
        double k_process_noise = this->get_parameter("kalman_process_noise").as_double();
        double k_measurement_noise = this->get_parameter("kalman_measurement_noise").as_double();
        double k_estimated_error = this->get_parameter("kalman_estimated_error").as_double();
        double k_initial_value = this->get_parameter("kalman_initial_value").as_double();

        kalman_surge = KalmanFilter(k_process_noise, k_measurement_noise, k_estimated_error, k_initial_value);
        kalman_yaw = KalmanFilter(k_process_noise, k_measurement_noise, k_estimated_error, k_initial_value);
        kalman_heave = KalmanFilter(k_process_noise, k_measurement_noise, k_estimated_error, k_initial_value);

        // Set Kalman filter thresholds
        kalman_surge.set_thresholds(measurement_threshold_, angular_threshold_);
        kalman_yaw.set_thresholds(measurement_threshold_, angular_threshold_);
        kalman_heave.set_thresholds(measurement_threshold_, angular_threshold_);

        // Initialize state variables
        velocity_surge = 0.0;
        velocity_yaw = 0.0;
        velocity_heave = 0.0;
        // velocity_heave = 0.0;
        target_vel_surge = 0.0;
        target_vel_yaw = 0.0;
        target_vel_heave = 0.0;
        // target_vel_heave = 0.0;
        last_time_ = this->now();

        // Log initialization
        RCLCPP_INFO(this->get_logger(), "Initializing BaseController with:");
        RCLCPP_INFO(this->get_logger(), "Kalman Filter: %s", use_kalman_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "PID Controller: %s", use_pid_ ? "ON" : "OFF");

        // Set up subscriptions
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&BaseController::twist_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", qos_profile,
            std::bind(&BaseController::imu_callback, this, std::placeholders::_1));

        // Set up publishers
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
        imu_alt_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_vel", 10);
        
        // Set up arm client
        arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // Arm the vehicle
        arm_vehicle(true);

        // Set up timer
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_rate), 
            std::bind(&BaseController::publish_rc_override, this));
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist &msg) {
        target_vel_surge = msg.linear.y;
        target_vel_yaw = msg.angular.z;
        target_vel_heave = msg.linear.z;
    }

    void imu_callback(const sensor_msgs::msg::Imu &msg) {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        RCLCPP_INFO(this->get_logger(), 
                    "Raw IMU \nSurge acc: %.2f, Yaw vel: %.2f, Heave acc: %.2f",
                    msg.linear_acceleration.y, - msg.angular_velocity.z, msg.linear_acceleration.z);

        double lin_acc_surge = -(msg.linear_acceleration.y - imu_surge_bias_);
        double ang_vel_yaw = - msg.angular_velocity.z;
        double lin_acc_heave = msg.linear_acceleration.z - imu_gravity_;

        double filtered_acc_surge, filtered_vel_yaw, filtered_acc_heave;
        if (use_kalman_) {
            filtered_acc_surge = kalman_surge.update(lin_acc_surge);
            filtered_vel_yaw = kalman_yaw.update(ang_vel_yaw);
            filtered_acc_heave = kalman_heave.update(lin_acc_heave);
        } else {
            filtered_acc_surge = lin_acc_surge;
            filtered_vel_yaw = ang_vel_yaw;
            filtered_acc_heave = lin_acc_heave;
        }

        RCLCPP_INFO(this->get_logger(),
                    "\nKalman magic \nSurge acc: %.2f->%.2f, Yaw vel %.2f->%.2f, Heave acc: %.2f->%.2f",
                    lin_acc_surge, filtered_acc_surge, ang_vel_yaw, filtered_vel_yaw, lin_acc_heave, filtered_acc_heave);

        // Integrate accelerations
        // velocity_surge += filtered_acc_surge * dt;
        // velocity_heave += filtered_acc_heave * dt;
        velocity_heave = filtered_acc_heave;
        velocity_yaw = filtered_vel_yaw;

        velocity_surge = filtered_acc_surge;
        velocity_yaw = velocity_yaw;
        // Publish IMU velocities
        auto imu_ref_msg = geometry_msgs::msg::Vector3();
        imu_ref_msg.x = velocity_surge;
        imu_ref_msg.y = velocity_yaw;
        imu_ref_msg.z = velocity_heave;
        imu_alt_publisher_->publish(imu_ref_msg);

        surge_pwm_ = compute_motor_pwm(target_vel_surge, velocity_surge,
                                        error_surge, integral_surge, previous_error_surge, dt);
        yaw_pwm_ = compute_motor_pwm(target_vel_yaw, velocity_yaw, 
                                    error_yaw, integral_yaw, previous_error_yaw, dt);
        heave_pwm_ = compute_motor_pwm(target_vel_heave, velocity_heave,
                                    error_heave, integral_heave, previous_error_heave, dt);

        // Publish current velocities

        RCLCPP_INFO(this->get_logger(),
                    "\nTarget_vel: [%.2f, %.2f, %.2f]\nMeasured_vel: [%.2f, %.2f,%.2f]\nPWM sent to thrusters: [%d, %d,%d]",
                    target_vel_surge, target_vel_yaw,target_vel_heave,
                    velocity_surge, velocity_yaw,velocity_heave,
                    surge_pwm_, yaw_pwm_, heave_pwm_);

        last_time_ = current_time;
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

    int compute_motor_pwm(double target_vel, double current_vel, 
                         double &error, double &integral, double &previous_error, double dt) {

        error = target_vel - current_vel;
        
        if (use_pid_ == 0) {
            int predicted_pwm = neutral_pwm_ + error * pwm_range_;
            if (std::abs(error * pwm_range_) < pwm_deadband_) {
            predicted_pwm = neutral_pwm_;
            }
            return std::clamp(predicted_pwm, neutral_pwm_ - pwm_range_, neutral_pwm_ + pwm_range_);
        }
        
        // If the target velocity and error are above the measurement threshold, update the integral term
        if (std::abs(target_vel) >= measurement_threshold_ && std::abs(error) >= measurement_threshold_) {
            integral += error * dt;
        } 
        else {
            integral = 0.0;
        }
        
        // Clamp the integral term to prevent integral windup
        integral = std::clamp(integral, -integral_clamp_, integral_clamp_);
        
        // Calculate the derivative term
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        
        int pwm_adjustment = static_cast<int>(
            kp_ * error * pwm_range_ +    // Proportional term
            ki_ * integral * pwm_range_ + // Integral term
            kd_ * derivative * pwm_range_ // Derivative term
        );
        
        // Calculate the predicted PWM value by adding the adjustment to the neutral PWM
        int predicted_pwm = neutral_pwm_ + pwm_adjustment;
        
        RCLCPP_INFO(this->get_logger(),
                "PID Debug: Error: %.2f, Integral: %.2f, Derivative: %.2f, PWM Adjustment: %d",
                error, integral, derivative, pwm_adjustment);
        // If the PWM adjustment is within the deadband, set the PWM to neutral
        if (std::abs(pwm_adjustment) < pwm_deadband_) {
            predicted_pwm = neutral_pwm_;
        }
        // Clamp the predicted PWM value to be within the allowed range
        return std::clamp(predicted_pwm, neutral_pwm_ - pwm_range_, neutral_pwm_ + pwm_range_);
    }

    // Parameters
    bool use_kalman_;
    bool use_pid_;
    int neutral_pwm_;
    int pwm_range_;
    double kp_, ki_, kd_;
    double imu_surge_bias_;
    double imu_gravity_;
    int pwm_deadband_;
    double integral_clamp_;
    double measurement_threshold_;
    double angular_threshold_;

    // State variables
    int heave_pwm_ = 1500;
    int surge_pwm_ = 1500;
    int yaw_pwm_ = 1500;
    double velocity_surge, velocity_yaw,velocity_heave;
    double target_vel_surge, target_vel_yaw,target_vel_heave;
    double error_surge{0}, error_yaw{0}, error_heave{0};
    double integral_surge{0}, integral_yaw{0},integral_heave{0};
    double previous_error_surge{0}, previous_error_yaw{0},previous_error_heave{0};
    
    // Time tracking
    rclcpp::Time last_time_;

    // Kalman filters
    KalmanFilter kalman_surge;
    KalmanFilter kalman_yaw;
    KalmanFilter kalman_heave;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    
    // Publishers
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_alt_publisher_;
    
    // Service client
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<BaseController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}
