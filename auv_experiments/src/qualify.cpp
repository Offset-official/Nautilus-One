#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Custom messages
#include "auv_interfaces/msg/detection_array.hpp"

// For publishing velocity commands
#include <geometry_msgs/msg/twist.hpp>

// C++ includes
#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class ExperimentNode : public rclcpp::Node
{
public:
  ExperimentNode()
  : Node("experiment_node")
  {
    RCLCPP_INFO(this->get_logger(), "ExperimentNode has started.");

    // Declare parameters with defaults
    gate_lost_threshold_ = this->declare_parameter("gate_lost_threshold", 3.0);
    forward_after_lost_   = this->declare_parameter("forward_after_lost", 13.0);
    pause_before_rotation_ = this->declare_parameter("pause_before_rotation", 2.0);  // wait period before rotating in LOST_GATE
    u_turn_duration_      = this->declare_parameter("u_turn_duration", 7.0);
    u_turn_yaw_speed_     = this->declare_parameter("u_turn_yaw_speed", 0.6);
    forward_speed_        = this->declare_parameter("forward_speed", 0.5);
    kp_yaw_               = this->declare_parameter("kp_yaw", 0.0005);
    kp_heave_             = this->declare_parameter("kp_heave", -0.01);
    x_tolerance_          = this->declare_parameter("x_tolerance", 30.0);
    y_tolerance_          = this->declare_parameter("y_tolerance", 30.0);
    big_error_threshold_  = this->declare_parameter("big_error_threshold", 50.0);

    // Subscribe to the /auv_camera/detections topic
    detections_sub_ = this->create_subscription<auv_interfaces::msg::DetectionArray>(
      "/auv_camera/detections", 10,
      std::bind(&ExperimentNode::detectionsCallback, this, std::placeholders::_1));

    // Publisher for /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for control loop (runs every 0.1 seconds)
    timer_ = this->create_wall_timer(100ms, std::bind(&ExperimentNode::controlLoop, this));

    // Initialization
    last_gate_detection_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Initialization done. Waiting for detections...");
  }

private:
  // -------------------------------------------------------
  // Movement States
  // -------------------------------------------------------
  enum MovementState
  {
    HORIZONTAL_ALIGNMENT = 0,
    VERTICAL_ALIGNMENT = 1,
    FORWARD_SURGE = 2,
    LOST_GATE = 3
  };

  // Helper to cycle to the next state (for the alignment cycle)
  MovementState getNextState(MovementState current)
  {
    switch (current) {
      case HORIZONTAL_ALIGNMENT:
        return VERTICAL_ALIGNMENT;
      case VERTICAL_ALIGNMENT:
        return FORWARD_SURGE;
      case FORWARD_SURGE:
        return HORIZONTAL_ALIGNMENT;
      default:
        return HORIZONTAL_ALIGNMENT;
    }
  }

  // -------------------------------------------------------
  // Callback: receives DetectionArray messages
  // -------------------------------------------------------
  void detectionsCallback(const auv_interfaces::msg::DetectionArray::SharedPtr msg)
  {
    bool found_gate = false;
    for (auto & detection : msg->detections) {
      if (detection.object == "Gate") {
        int center_x = (detection.x1 + detection.x2) / 2;
        int center_y = (detection.y1 + detection.y2) / 2;
        gate_center_ = std::make_pair(center_x, center_y);
        last_gate_detection_time_ = this->now();
        RCLCPP_INFO(
          this->get_logger(), "Gate found at center: (%d, %d). Updated last_gate_detection_time_.",
          center_x, center_y);
        found_gate = true;
        break;  // process only the first gate detection
      }
    }
    if (!found_gate) {
      RCLCPP_INFO(this->get_logger(), "No Gate found in this DetectionArray.");
    }
  }

  // -------------------------------------------------------
  // Timer Callback: main control loop
  // -------------------------------------------------------
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    // Initialize all velocities to zero
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    auto now_time = this->now();
    double elapsed_since_gate = (now_time - last_gate_detection_time_).seconds();

    // If the gate is detected recently and we are not in LOST_GATE, run the alignment FSM
    if (elapsed_since_gate < gate_lost_threshold_ && movement_state_ != LOST_GATE) {
      cmd_vel_msg = doAlignmentLogic();
    } else {
      // Switch to LOST_GATE if not already in that state
      if (movement_state_ != LOST_GATE) {
        movement_state_ = LOST_GATE;
        lost_gate_start_time_ = now_time;
        RCLCPP_WARN(this->get_logger(), "[FSM] Gate is lost! Switching to LOST_GATE state.");
      }
      // Calculate time spent in LOST_GATE state
      double elapsed_lost_state = (now_time - lost_gate_start_time_).seconds();
      // LOST_GATE phases:
      // Phase 1: Forward movement for forward_after_lost_ seconds
      if (elapsed_lost_state < forward_after_lost_) {
        cmd_vel_msg.linear.x = forward_speed_;
        cmd_vel_msg.angular.z = 0.0;
        RCLCPP_INFO(
          this->get_logger(), "[LOST_GATE] Moving forward for %.2f sec (elapsed=%.2f/%.2f)",
          forward_after_lost_, elapsed_lost_state, forward_after_lost_);
      }
      // Phase 2: Wait (stop) for pause_before_rotation_ seconds before beginning the rotation
      else if (elapsed_lost_state < (forward_after_lost_ + pause_before_rotation_)) {
        // All velocities remain zero (pause)
        RCLCPP_INFO(
          this->get_logger(), "[LOST_GATE] Waiting for %.2f sec before rotating (elapsed=%.2f/%.2f)",
          pause_before_rotation_, elapsed_lost_state - forward_after_lost_, pause_before_rotation_);
      }
      // Phase 3: Rotate for u_turn_duration_ seconds
      else if (elapsed_lost_state < (forward_after_lost_ + pause_before_rotation_ + u_turn_duration_)) {
        cmd_vel_msg.angular.z = u_turn_yaw_speed_;
        RCLCPP_INFO(
          this->get_logger(),
          "[LOST_GATE] Rotating for %.2f sec (elapsed=%.2f/%.2f), speed=%.2f rad/s",
          u_turn_duration_, elapsed_lost_state - forward_after_lost_ - pause_before_rotation_,
          u_turn_duration_, u_turn_yaw_speed_);
      }
      // Phase 4: After the defined phases, keep rotating until the gate is found
      else {
        cmd_vel_msg.angular.z = u_turn_yaw_speed_;
        RCLCPP_INFO(
          this->get_logger(), "[LOST_GATE] Continuing rotation until gate is reacquired.");
      }

      // If the gate is seen again, revert back to horizontal alignment immediately
      if (elapsed_since_gate < gate_lost_threshold_) {
        RCLCPP_INFO(this->get_logger(), "[LOST_GATE] Gate reacquired! Switching back to HORIZONTAL_ALIGNMENT.");
        movement_state_ = HORIZONTAL_ALIGNMENT;
      }
    }

    // Publish the computed command velocity
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // doAlignmentLogic: runs the horizontal->vertical->forward cyclical alignment
  // -------------------------------------------------------
  geometry_msgs::msg::Twist doAlignmentLogic()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    // Typical image center coordinates (assumed image size: 640x480)
    float image_center_x = 320.0f;
    float image_center_y = 240.0f;

    float gate_x = static_cast<float>(gate_center_.first);
    float gate_y = static_cast<float>(gate_center_.second);

    // Errors in image coordinates:
    float error_x = gate_x - image_center_x;  // horizontal error
    float error_y = gate_y - image_center_y;  // vertical error

    switch (movement_state_) {
      case HORIZONTAL_ALIGNMENT: {
          // Yaw to correct horizontal error
          float yaw_cmd = kp_yaw_ * error_x;
          cmd_vel_msg.angular.z = yaw_cmd;
          // Maintain a small forward movement
          cmd_vel_msg.linear.x = 0.2f;
          if (std::fabs(error_x) < x_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "[FSM] Horizontal alignment achieved. Switching to VERTICAL_ALIGNMENT immediately.");
            movement_state_ = VERTICAL_ALIGNMENT;
          }
          break;
        }
      case VERTICAL_ALIGNMENT: {
          // Command vertical motion (heave) to correct vertical error
          float z_cmd = kp_heave_ * error_y;
          cmd_vel_msg.linear.z = z_cmd;
          // Maintain a small forward movement
          cmd_vel_msg.linear.x = 0.2f;
          if (std::fabs(error_y) < y_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "[FSM] Vertical alignment achieved. Switching to FORWARD_SURGE immediately.");
            movement_state_ = FORWARD_SURGE;
          }
          break;
        }
      case FORWARD_SURGE: {
          // Surge forward at a higher speed
          cmd_vel_msg.linear.x = 1.0f;
          // If the gate drifts too far from center, revert to alignment states
          if (std::fabs(error_x) > big_error_threshold_) {
            RCLCPP_INFO(this->get_logger(), "[FSM] Significant horizontal drift => switching back to HORIZONTAL_ALIGNMENT.");
            movement_state_ = HORIZONTAL_ALIGNMENT;
          } else if (std::fabs(error_y) > big_error_threshold_) {
            RCLCPP_INFO(this->get_logger(), "[FSM] Significant vertical drift => switching back to VERTICAL_ALIGNMENT.");
            movement_state_ = VERTICAL_ALIGNMENT;
          }
          break;
        }
      case LOST_GATE: {
          // This branch should not be reached in normal alignment logic.
          cmd_vel_msg.linear.x = 0.0;
          cmd_vel_msg.angular.z = 0.0;
          break;
        }
      default:
        break;
    }

    // Debug info: log current state and errors
    RCLCPP_INFO(
      this->get_logger(), "state=%d | error_x=%.1f, error_y=%.1f => forward=%.2f, z=%.2f, yaw=%.4f",
      movement_state_, error_x, error_y, cmd_vel_msg.linear.x, cmd_vel_msg.linear.z,
      cmd_vel_msg.angular.z);

    // (Preserve any swapping logic if needed; here we keep the same order as before)
    std::swap(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y);
    return cmd_vel_msg;
  }

  // -------------------------------------------------------
  // Member variables & parameters
  // -------------------------------------------------------
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Gate center in image coordinates
  std::pair<int, int> gate_center_{320, 240};
  rclcpp::Time last_gate_detection_time_;
  rclcpp::Time lost_gate_start_time_;

  // FSM current state
  MovementState movement_state_{HORIZONTAL_ALIGNMENT};

  // Parameters (all configurable via ROS parameters)
  double gate_lost_threshold_;
  double forward_after_lost_;
  double pause_before_rotation_;
  double u_turn_duration_;
  double u_turn_yaw_speed_;
  double forward_speed_;
  double kp_yaw_;
  double kp_heave_;
  double x_tolerance_;
  double y_tolerance_;
  double big_error_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
