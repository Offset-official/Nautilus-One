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
#include <limits>

using namespace std::chrono_literals;

class ExperimentNode : public rclcpp::Node
{
public:
  ExperimentNode()
  : Node("experiment_node")
  {
    RCLCPP_INFO(this->get_logger(), "ExperimentNode has started.");

    // 1) Subscribe to the /auv_camera/detections topic
    detections_sub_ = this->create_subscription<auv_interfaces::msg::DetectionArray>(
      "/auv_camera_front/detections", 10,
      std::bind(&ExperimentNode::detectionsCallback, this, std::placeholders::_1));

    // 2) Publisher for /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 3) Timer for control loop (runs every 0.1 seconds)
    timer_ = this->create_wall_timer(100ms, std::bind(&ExperimentNode::controlLoop, this));

    // Initialize gate center to camera center (360, 640)
    gate_center_ = std::make_pair(360, 640);
    last_gate_detection_time_ = this->now();

    // Initialize FSM state to SEARCHING so that the robot rotates to find the gate.
    movement_state_ = SEARCHING;
    lost_count_ = 0;
    RCLCPP_INFO(this->get_logger(), "Initialization done. Starting in SEARCHING mode.");
  }

private:
  // -------------------------------------------------------
  // FSM States for movement
  // -------------------------------------------------------
  enum MovementState
  {
    SEARCHING = 0,             // Rotate in place to look for gate
    HORIZONTAL_ALIGNMENT = 1,  // Apply horizontal (yaw) correction while moving slowly forward
    FORWARD_SURGE = 2,         // Move forward quickly (0.5) once aligned
    GATE_LOST_FORWARD = 3,     // When gate is lost, move forward for 13s
    U_TURN = 4,                // Execute U-turn until gate is found again
    STOP = 5                   // Final state: after second loss, drive forward 13s then stop
  };

  // -------------------------------------------------------
  // Callback: receives DetectionArray messages
  // -------------------------------------------------------
  void detectionsCallback(const auv_interfaces::msg::DetectionArray::SharedPtr msg)
  {
    bool found_gate = false;
    float max_confidence = -std::numeric_limits<float>::infinity();
    // Assuming your detection message has a 'confidence' field.
    auv_interfaces::msg::Detection best_detection;

    for (auto &detection : msg->detections) {
      if (detection.object == "Gate") {
        if (detection.confidence > max_confidence) {
          best_detection = detection;
          max_confidence = detection.confidence;
          found_gate = true;
        }
      }
    }

    if (found_gate) {
      int center_x = (best_detection.x1 + best_detection.x2) / 2;
      int center_y = (best_detection.y1 + best_detection.y2) / 2;
      gate_center_ = std::make_pair(center_x, center_y);
      last_gate_detection_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Gate found at center: (%d, %d) with confidence %.2f.",
                  center_x, center_y, max_confidence);
    } else {
      RCLCPP_INFO(this->get_logger(), "No Gate found in this DetectionArray.");
    }
  }

  // -------------------------------------------------------
  // Timer Callback: main control loop implementing the FSM
  // -------------------------------------------------------
  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    // Reset unused components
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;

    auto now_time = this->now();
    // Calculate elapsed time since last gate detection
    double dt = (now_time - last_gate_detection_time_).seconds();
    // Use a 3-second threshold to consider the gate lost.
    const double GATE_LOST_THRESHOLD = 3.0;
    bool gate_lost = (dt >= GATE_LOST_THRESHOLD);

    // Define the camera center
    float image_center_x = 360.0f;
    // Compute horizontal error only if the gate is not lost.
    float error_x = (!gate_lost) ? static_cast<float>(gate_center_.first) - image_center_x : 0.0f;

    // FSM Logic
    switch (movement_state_) {
      case SEARCHING: {
        // In SEARCHING state, rotate in place (anticlockwise => negative angular.z) to look for the gate.
        if (!gate_lost) {
          RCLCPP_INFO(this->get_logger(), "[FSM] Gate detected. Switching to HORIZONTAL_ALIGNMENT.");
          movement_state_ = HORIZONTAL_ALIGNMENT;
        } else {
          cmd_vel_msg.angular.z = 1;
          cmd_vel_msg.linear.y = 0.0;
          RCLCPP_INFO(this->get_logger(), "[SEARCHING] Rotating to find gate.");
        }
        break;
      }
      case HORIZONTAL_ALIGNMENT: {
        if (gate_lost) {
          // If the gate is lost during alignment, transition to GATE_LOST_FORWARD.
          movement_state_ = GATE_LOST_FORWARD;
          state_start_time_ = now_time;
          lost_count_++;
          RCLCPP_WARN(this->get_logger(), "[FSM] Gate lost during HORIZONTAL_ALIGNMENT, switching to GATE_LOST_FORWARD.");
        } else {
          // Use a constant yaw correction of 0.5 (with proper sign: negative to turn anticlockwise).
          float tolerance = 30.0f;
          if (std::fabs(error_x) > tolerance) {
            // If the gate is to the right (error > 0), turn anticlockwise (angular.z negative).
            // If the gate is to the left (error < 0), turn clockwise (angular.z positive).
            cmd_vel_msg.angular.z = (error_x < 0) ? -1 : 1;
            cmd_vel_msg.linear.y = 0.0;  // stay in place while aligning
            RCLCPP_INFO(this->get_logger(), "[HORIZONTAL_ALIGNMENT] Correcting: error=%.1f", error_x);
          } else {
            RCLCPP_INFO(this->get_logger(), "[FSM] Alignment achieved. Switching to FORWARD_SURGE.");
            movement_state_ = FORWARD_SURGE;
          }
        }
        break;
      }
      case FORWARD_SURGE: {
        if (gate_lost) {
          movement_state_ = GATE_LOST_FORWARD;
          state_start_time_ = now_time;
          lost_count_++;
          RCLCPP_WARN(this->get_logger(), "[FSM] Gate lost during FORWARD_SURGE, switching to GATE_LOST_FORWARD.");
        } else {
          // If a significant drift occurs, return to alignment.
          if (std::fabs(error_x) > 50.0f) {
            RCLCPP_INFO(this->get_logger(), "[FSM] Significant drift detected. Switching back to HORIZONTAL_ALIGNMENT.");
            movement_state_ = HORIZONTAL_ALIGNMENT;
          }
          cmd_vel_msg.linear.y = 1;  // surge forward
          cmd_vel_msg.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "[FORWARD_SURGE] Moving forward.");
        }
        break;
      }
      case GATE_LOST_FORWARD: {
        // Continue driving forward at 0.5 when the gate is lost.
        cmd_vel_msg.linear.y = 1;
        cmd_vel_msg.angular.z = 0.0;
        if ((now_time - state_start_time_).seconds() >= 13.0) {
          if (lost_count_ == 1) {
            movement_state_ = U_TURN;
            RCLCPP_INFO(this->get_logger(), "[FSM] 13 seconds elapsed. Switching to U_TURN.");
          } else if (lost_count_ >= 2) {
            movement_state_ = STOP;
            state_start_time_ = now_time; // record start time for final stop timer
            RCLCPP_INFO(this->get_logger(), "[FSM] 13 seconds elapsed. Switching to STOP.");
          }
        }
        break;
      }
      case U_TURN: {
        // Execute U-turn: rotate anticlockwise (angular.z negative) until a gate is detected.
        cmd_vel_msg.angular.z = 1;
        cmd_vel_msg.linear.y = 0.0;
        RCLCPP_INFO(this->get_logger(), "[U_TURN] Executing U-turn.");
        if (!gate_lost) {
          // Once the gate is detected again, reset lost counter and switch back to alignment.
          lost_count_ = 0;
          movement_state_ = HORIZONTAL_ALIGNMENT;
          RCLCPP_INFO(this->get_logger(), "[FSM] Gate re-acquired during U-turn. Switching to HORIZONTAL_ALIGNMENT.");
        }
        break;
      }
      case STOP: {
        // In STOP state, drive forward for 13 seconds then come to a halt.
        if ((now_time - state_start_time_).seconds() < 13.0) {
          cmd_vel_msg.linear.y = 1;
          cmd_vel_msg.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "[STOP] Moving forward during final 13 seconds.");
        } else {
          cmd_vel_msg.linear.y = 0.0;
          cmd_vel_msg.angular.z = 0.0;
          RCLCPP_INFO(this->get_logger(), "[STOP] Final stop achieved.");
        }
        break;
      }
      default:
        break;
    }

    // Publish the command velocity
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // Member variables
  // -------------------------------------------------------
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Track the gate center from detections (in image coordinates)
  std::pair<int, int> gate_center_;
  // Track when we last saw the gate
  rclcpp::Time last_gate_detection_time_;

  // FSM state and auxiliary variables
  MovementState movement_state_;
  rclcpp::Time state_start_time_;
  int lost_count_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
