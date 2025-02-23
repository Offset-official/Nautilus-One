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
  ExperimentNode() : Node("experiment_node")
  {
    RCLCPP_INFO(this->get_logger(), "ExperimentNode has started.");

    // 1) Subscribe to the /auv_camera/detections topic
    detections_sub_ = this->create_subscription<auv_interfaces::msg::DetectionArray>(
      "/auv_camera/detections", 10,
      std::bind(&ExperimentNode::detectionsCallback, this, std::placeholders::_1));

    // 2) Publisher for /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 3) Timer for control loop (runs every 0.1 seconds)
    timer_ = this->create_wall_timer(100ms, std::bind(&ExperimentNode::controlLoop, this));

    // Initialize
    last_gate_detection_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Initialization done. Waiting for detections...");
  }

private:
  // -------------------------------------------------------
  // Movement States
  // -------------------------------------------------------
  enum MovementState {
    HORIZONTAL_ALIGNMENT = 0,
    VERTICAL_ALIGNMENT = 1,
    FORWARD_SURGE = 2,
    LOST_GATE = 3
  };

  // Helper to cycle to the next state
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
      // Check if this detection is "Gate"
      if (detection.object == "Gate") {
        // Compute the center of bounding box in image coordinates
        int center_x = (detection.x1 + detection.x2) / 2;
        int center_y = (detection.y1 + detection.y2) / 2;

        // Update our stored gate center
        gate_center_ = std::make_pair(center_x, center_y);

        // Update the time we last saw the gate
        last_gate_detection_time_ = this->now();

        RCLCPP_INFO(
          this->get_logger(), "Gate found at center: (%d, %d). Updated last_gate_detection_time_.",
          center_x, center_y);

        found_gate = true;
        break;  // Stop at the first "Gate" detection
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
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;  // not used
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    // Calculate how long since we last saw the gate
    auto now_time = this->now();
    double elapsed_since_gate = (now_time - last_gate_detection_time_).seconds();

    // Constants
    const double GATE_LOST_THRESHOLD = 3.0;  // after 3s, consider gate "lost"
    const double FORWARD_AFTER_LOST = 13.0;  // go forward for 13s
    const double U_TURN_DURATION = 7.0;      // then yaw in place for 6s
    const double U_TURN_YAW_SPEED = 0.6;     // how fast to rotate (rad/s)
    const double FORWARD_SPEED = 0.5;        // forward speed while "lost" briefly

    // If we see the gate "recently," do normal cyclical alignment
    if (elapsed_since_gate < GATE_LOST_THRESHOLD && movement_state_ != LOST_GATE) {
      // Normal cyclical alignment
      cmd_vel_msg = doAlignmentLogic();
    } else {
      // If we haven't switched to LOST_GATE yet, do it now
      if (movement_state_ != LOST_GATE) {
        movement_state_ = LOST_GATE;
        lost_gate_start_time_ = now_time;  // mark when we entered LOST_GATE
        RCLCPP_WARN(this->get_logger(), "[FSM] Gate is lost! Switching to LOST_GATE state.");
      }

      // Once in LOST_GATE, figure out how long we've been here
      double elapsed_lost_state = (now_time - lost_gate_start_time_).seconds();

      // First 13s => go forward
      if (elapsed_lost_state < FORWARD_AFTER_LOST) {
        cmd_vel_msg.linear.x = FORWARD_SPEED;
        cmd_vel_msg.angular.z = 0.0;
        RCLCPP_INFO(
          this->get_logger(), "[LOST_GATE] Running forward for 13s (elapsed=%.2f/%.2f)",
          elapsed_lost_state, FORWARD_AFTER_LOST);
      }
      // Then next 6s => rotate (U-turn)
      else if (elapsed_lost_state < (FORWARD_AFTER_LOST + U_TURN_DURATION)) {
        cmd_vel_msg.angular.z = U_TURN_YAW_SPEED;
        RCLCPP_INFO(
          this->get_logger(),
          "[LOST_GATE] Rotating in place for 6s (elapsed=%.2f/%.2f), speed=%.2f rad/s",
          elapsed_lost_state - FORWARD_AFTER_LOST, U_TURN_DURATION, U_TURN_YAW_SPEED);
      } else {
        // After 19s total in LOST_GATE, keep rotating in place indefinitely
        // (until we see the gate again, which will break us out of LOST_GATE).
        cmd_vel_msg.angular.z = U_TURN_YAW_SPEED;
        RCLCPP_INFO(
          this->get_logger(),
          "[LOST_GATE] Past forward+turn phases => keep rotating until gate found.");
      }

      // If we happen to see the gate again (meaning elapsed_since_gate < GATE_LOST_THRESHOLD),
      // go back to alignment.
      if (elapsed_since_gate < GATE_LOST_THRESHOLD) {
        RCLCPP_INFO(
          this->get_logger(), "[LOST_GATE] Gate reacquired => back to horizontal alignment!");
        movement_state_ = HORIZONTAL_ALIGNMENT;
      }
    }

    // Publish command
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // doAlignmentLogic:
  //   runs the horizontal->vertical->forward cyclical alignment
  //   returns the cmd_vel
  // -------------------------------------------------------
  geometry_msgs::msg::Twist doAlignmentLogic()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;

    float image_center_x = 320.0f;  // half of typical 640 width
    float image_center_y = 240.0f;  // half of typical 480 height

    float gate_x = static_cast<float>(gate_center_.first);
    float gate_y = static_cast<float>(gate_center_.second);

    // Errors in image coordinates:
    float error_x = gate_x - image_center_x;  // horizontal offset
    float error_y = gate_y - image_center_y;  // vertical offset

    // Gains
    float kp_yaw = 0.0005f;   // for horizontal alignment
    float kp_heave = -0.01f;  // for vertical alignment (negative if up is smaller y)

    // Tolerances
    float x_tolerance = 30.0f;
    float y_tolerance = 30.0f;

    switch (movement_state_) {
      case HORIZONTAL_ALIGNMENT: {
        // Yaw to fix horizontal offset
        float yaw_cmd = kp_yaw * error_x;
        cmd_vel_msg.angular.z = yaw_cmd;
        // Keep creeping forward a bit
        cmd_vel_msg.linear.x = 0.2f;
        cmd_vel_msg.linear.z = 0.0f;

        if (std::fabs(error_x) < x_tolerance) {
          RCLCPP_INFO(
            this->get_logger(), "[FSM] Horizontal alignment achieved. Next => VERTICAL_ALIGNMENT.");
          movement_state_ = getNextState(movement_state_);
        }
        break;
      }
      case VERTICAL_ALIGNMENT: {
        // Use heave to fix vertical offset
        float z_cmd = kp_heave * error_y;
        cmd_vel_msg.linear.z = z_cmd;
        // Keep creeping forward a bit
        cmd_vel_msg.linear.x = 0.2f;
        cmd_vel_msg.angular.z = 0.0f;

        if (std::fabs(error_y) < y_tolerance) {
          RCLCPP_INFO(
            this->get_logger(), "[FSM] Vertical alignment achieved. Next => FORWARD_SURGE.");
          movement_state_ = getNextState(movement_state_);
        }
        break;
      }
      case FORWARD_SURGE: {
        // Surge forward
        cmd_vel_msg.linear.x = 1.0f;
        cmd_vel_msg.angular.z = 0.0f;
        cmd_vel_msg.linear.z = 0.0f;

        // If the gate drifts away in x or y, go back to alignment
        float big_error_threshold = 50.0f;
        if (std::fabs(error_x) > big_error_threshold) {
          RCLCPP_INFO(
            this->get_logger(), "[FSM] Drifted horizontally => back to HORIZONTAL_ALIGNMENT.");
          movement_state_ = HORIZONTAL_ALIGNMENT;
        } else if (std::fabs(error_y) > big_error_threshold) {
          RCLCPP_INFO(
            this->get_logger(), "[FSM] Drifted vertically => back to VERTICAL_ALIGNMENT.");
          movement_state_ = VERTICAL_ALIGNMENT;
        }
        break;
      }
      case LOST_GATE: {
        // Normally we don't call this function in LOST_GATE,
        // but if we do, let's just return no movement:
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        break;
      }
    }

    // Debug info
    RCLCPP_INFO(
      this->get_logger(), "state=%d | err_x=%.1f err_y=%.1f => surge=%.2f, z=%.2f, yaw=%.4f",
      movement_state_, error_x, error_y, cmd_vel_msg.linear.x, cmd_vel_msg.linear.z,
      cmd_vel_msg.angular.z);
    std::swap(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y);
    return cmd_vel_msg;
  }

  // -------------------------------------------------------
  // Member variables
  // -------------------------------------------------------
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Track the gate center from detections (in image coords).
  std::pair<int, int> gate_center_{320, 240};

  // Track when we last saw the Gate
  rclcpp::Time last_gate_detection_time_;

  // FSM: cycle among HORIZONTAL, VERTICAL, FORWARD, plus LOST_GATE
  MovementState movement_state_{HORIZONTAL_ALIGNMENT};

  // Time when we first entered LOST_GATE
  rclcpp::Time lost_gate_start_time_;
};

// -------------------------------------------------------
// main: create and spin the node
// -------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExperimentNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
