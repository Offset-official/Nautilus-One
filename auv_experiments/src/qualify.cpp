#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Custom messages
#include "auv_interfaces/msg/detection_array.hpp"

// For publishing velocity commands
#include <geometry_msgs/msg/twist.hpp>

// C++ includes
#include <vector>
#include <chrono>
#include <cmath>

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
      "/auv_camera/detections",
      10,
      std::bind(&ExperimentNode::detectionsCallback, this, std::placeholders::_1)
    );

    // 2) Publisher for /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 3) Timer for control loop (runs every 0.1 seconds)
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&ExperimentNode::controlLoop, this)
    );

    // Initialize the time we last detected the gate
    last_gate_detection_time_ = this->now();
  }

private:
  // -------------------------------------------------------
  // Movement States: We cycle through them: H->V->F->H->...
  // -------------------------------------------------------
  enum MovementState
  {
    HORIZONTAL_ALIGNMENT = 0,
    VERTICAL_ALIGNMENT   = 1,
    FORWARD_SURGE        = 2
  };

  // Helper to cycle to the next state
  MovementState getNextState(MovementState current)
  {
    switch (current)
    {
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
    // Print how many detections we received
    RCLCPP_INFO(this->get_logger(),
                "detectionsCallback: received %zu detections",
                msg->detections.size());

    bool found_gate = false;

    for (auto &detection : msg->detections)
    {
      // Check if this detection is "Gate"
      if (detection.object == "Gate")
      {
        // Compute the center of bounding box in image coordinates
        int center_x = (detection.x1 + detection.x2) / 2;
        int center_y = (detection.y1 + detection.y2) / 2;

        // Update our stored gate center
        gate_center_ = std::make_pair(center_x, center_y);

        // Update the time we last saw the gate
        last_gate_detection_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "Gate found at center: (%d, %d). Updated last_gate_detection_time_.",
                    center_x, center_y);

        found_gate = true;
        // If multiple "Gate" detections are possible, stop at the first one
        break;
      }
    }

    if (!found_gate)
    {
      // If we didn't find a Gate in this message, log it
      RCLCPP_INFO(this->get_logger(),
                  "No Gate found in this message. Continuing ...");
    }
  }

  // -------------------------------------------------------
  // Timer Callback: main control loop
  // -------------------------------------------------------
  void controlLoop()
  {
    RCLCPP_DEBUG(this->get_logger(), "Running controlLoop()...");

    // Initialize cmd_vel with all zeros
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x  = 0.0;  // surge
    cmd_vel_msg.linear.y  = 0.0;  // not used
    cmd_vel_msg.linear.z  = 0.0;  // heave
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;  // yaw

    // Calculate how long since we last saw the gate
    auto now_time = this->now();
    double elapsed_since_gate = (now_time - last_gate_detection_time_).seconds();

    /*
      Gate-lost logic:
        1) If we haven't detected gate for more than GATE_LOST_THRESHOLD,
           we consider it "lost."
        2) Keep going forward for KEEP_GOING_DURATION seconds after losing it,
           then stop.
    */
    const double GATE_LOST_THRESHOLD = 3.0;
    const double KEEP_GOING_DURATION  = 6.0;

    // If gate still "visible" or recently seen
    if (elapsed_since_gate < GATE_LOST_THRESHOLD)
    {
      // -----------------------------------------------------
      // CASE 1: Gate is visible => do the cyclical alignment
      // -----------------------------------------------------
      float image_center_x = 320.0f;  // half of typical 640 width
      float image_center_y = 240.0f;  // half of typical 480 height

      // Gate center from detection callback
      float gate_x = static_cast<float>(gate_center_.first);
      float gate_y = static_cast<float>(gate_center_.second);

      // Errors in image coordinates:
      float error_x = gate_x - image_center_x;  // horizontal offset
      float error_y = gate_y - image_center_y;  // vertical offset

      // Gains
      // Yaw for horizontal alignment
      float kp_yaw   = 0.0005f;  
      // Heave for vertical alignment
      float kp_heave = -0.01f;  // might need to flip sign if up/down is reversed

      // Tolerances: how close is "good enough" for each alignment step
      float x_tolerance = 30.0f;
      float y_tolerance = 30.0f;

      // Evaluate current state in cyclical FSM
      switch (movement_state_)
      {
        case HORIZONTAL_ALIGNMENT:
        {
          // Yaw to fix horizontal offset
          float yaw_cmd = kp_yaw * error_x;
          cmd_vel_msg.angular.z = yaw_cmd;

          // Optionally add a small forward surge so you don't stand still
          cmd_vel_msg.linear.x = 0.2f;  // creeping forward

          // No vertical movement in this state
          cmd_vel_msg.linear.z = 0.0f;

          // If horizontally aligned, go to next state => VERTICAL
          if (std::fabs(error_x) < x_tolerance)
          {
            RCLCPP_INFO(this->get_logger(),
                        "[FSM] Horizontal alignment achieved. Next => VERTICAL_ALIGNMENT.");
            movement_state_ = getNextState(movement_state_);
          }
          break;
        }

        case VERTICAL_ALIGNMENT:
        {
          // Use heave (z) to fix vertical offset
          float z_cmd = kp_heave * error_y;
          cmd_vel_msg.linear.z = z_cmd;

          // Possibly keep creeping forward
          cmd_vel_msg.linear.x = 0.2f;

          // No yaw in this state
          cmd_vel_msg.angular.z = 0.0f;

          // If vertically aligned, go to next state => FORWARD
          if (std::fabs(error_y) < y_tolerance)
          {
            RCLCPP_INFO(this->get_logger(),
                        "[FSM] Vertical alignment achieved. Next => FORWARD_SURGE.");
            movement_state_ = getNextState(movement_state_);
          }
          break;
        }

        case FORWARD_SURGE:
        {
          // Surge forward
          cmd_vel_msg.linear.x = 1.0f;  // go forward
          cmd_vel_msg.angular.z = 0.0f;
          cmd_vel_msg.linear.z  = 0.0f;

          // We might define a time or some condition to switch back
          // but here let's say after a short distance or time, we cycle back.
          // For simplicity, let's check if horizontal or vertical error is
          // large again => go back to HORIZONTAL. That effectively keeps us
          // realigning if the gate "moves" in the image.

          float big_error_threshold = 50.0f; // a bit bigger than x_tolerance
          if (std::fabs(error_x) > big_error_threshold)
          {
            RCLCPP_INFO(this->get_logger(),
                        "[FSM] We drifted horizontally => back to HORIZONTAL_ALIGNMENT.");
            movement_state_ = HORIZONTAL_ALIGNMENT;
          }
          else if (std::fabs(error_y) > big_error_threshold)
          {
            RCLCPP_INFO(this->get_logger(),
                        "[FSM] We drifted vertically => back to VERTICAL_ALIGNMENT.");
            movement_state_ = VERTICAL_ALIGNMENT;
          }

          // Alternatively, you could just forcibly cycle forward->horizontal->vertical:
          //   movement_state_ = getNextState(movement_state_);
          //   after a time or some condition
          break;
        }
      }

      // Debug info
      RCLCPP_INFO(this->get_logger(),
                  "Gate visible. state=%d | err_x=%.1f err_y=%.1f => surge=%.2f heave=%.2f yaw=%.4f",
                  movement_state_, error_x, error_y,
                  cmd_vel_msg.linear.x, cmd_vel_msg.linear.z, cmd_vel_msg.angular.z);
    }
    else if (elapsed_since_gate < (GATE_LOST_THRESHOLD + KEEP_GOING_DURATION))
    {
      // -----------------------------------------------------
      // CASE 2: Gate recently lost => keep going forward
      // -----------------------------------------------------
      cmd_vel_msg.linear.x  = 0.5; // forward
      cmd_vel_msg.linear.z  = 0.0;
      cmd_vel_msg.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(),
                  "Gate lost for %.2f s => continuing forward (x=0.5) for up to %.1f s.",
                  elapsed_since_gate, KEEP_GOING_DURATION);
    }
    else
    {
      // -----------------------------------------------------
      // CASE 3: Gate lost for a long time => stop
      // -----------------------------------------------------
      cmd_vel_msg.linear.x  = 0.0;
      cmd_vel_msg.linear.z  = 0.0;
      cmd_vel_msg.angular.z = 0.0;

      RCLCPP_INFO(this->get_logger(),
                  "Gate lost for %.2f s => stopping now.",
                  elapsed_since_gate);
    }

    // Publish our command
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // Member variables
  // -------------------------------------------------------
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Track the gate center from detections (in image coords).
  // Start in the image center to avoid large initial error.
  std::pair<int,int> gate_center_{320, 240};

  // Track when we last saw the Gate
  rclcpp::Time last_gate_detection_time_;

  // FSM: we cycle among HORIZONTAL, VERTICAL, FORWARD
  MovementState movement_state_{HORIZONTAL_ALIGNMENT};

  // If you want to time the forward motion, store time here
  rclcpp::Time pass_through_start_time_;
};

// -------------------------------------------------------
// main: create and spin the node
// -------------------------------------------------------
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExperimentNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
