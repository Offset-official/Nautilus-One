#include "auv_interfaces/action/depth_descent.hpp"
#include "auv_interfaces/msg/detection_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;
using DepthDescent = auv_interfaces::action::DepthDescent;
using GoalHandleDepthDescent = rclcpp_action::ClientGoalHandle<DepthDescent>;
class ExperimentNode : public rclcpp::Node {
public:
  ExperimentNode() : Node("experiment_node") {
    RCLCPP_INFO(this->get_logger(), "ExperimentNode has started.");

    gate_lost_threshold_ = this->declare_parameter("gate_lost_threshold", 3.0);
    forward_after_lost_ = this->declare_parameter("forward_after_lost", 13.0);
    pause_before_rotation_ =
        this->declare_parameter("pause_before_rotation", 2.0);
    u_turn_duration_ = this->declare_parameter("u_turn_duration", 7.0);
    u_turn_yaw_speed_ = this->declare_parameter("u_turn_yaw_speed", 0.6);
    forward_speed_ = this->declare_parameter("forward_speed", 0.5);
    kp_yaw_ = this->declare_parameter("kp_yaw", 0.005);
    kp_heave_ = this->declare_parameter("kp_heave", -0.01);
    x_tolerance_ = this->declare_parameter("x_tolerance", 30.0);
    y_tolerance_ = this->declare_parameter("y_tolerance", 30.0);
    big_error_threshold_ = this->declare_parameter("big_error_threshold", 50.0);

    detections_sub_ =
        this->create_subscription<auv_interfaces::msg::DetectionArray>(
            "/auv_camera_front/detections", 10,
            std::bind(&ExperimentNode::detectionsCallback, this,
                      std::placeholders::_1));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    soft_arm_client_ =
        this->create_client<std_srvs::srv::SetBool>("/soft_arm/");

    // Initialize the depth descent action client in the constructor
    depth_descent_action =
        rclcpp_action::create_client<DepthDescent>(this, "depth_descent");

    // Wait for 30 seconds before starting the control loop
    RCLCPP_INFO(this->get_logger(),
                "Waiting 30 seconds before starting control loop...");
    startup_timer_ = this->create_wall_timer(30s, [this]() {
      RCLCPP_INFO(this->get_logger(),
                  "30-second wait completed. Starting control loop.");
      // Cancel the startup timer as it's no longer needed
      startup_timer_->cancel();

      // call the soft arm
      auto arm_request = std::make_shared<std_srvs::srv::SetBool::Request>();
      arm_request->data = true;
      soft_arm_client_->async_send_request(arm_request);

      // descend the depth
      auto descent_goal = DepthDescent::Goal();
      descent_goal.target_depth = 1.0; // Set your desired target depth here

      auto send_goal_options =
          rclcpp_action::Client<DepthDescent>::SendGoalOptions();
      send_goal_options.goal_response_callback =
          std::bind(&ExperimentNode::goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
          std::bind(&ExperimentNode::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
          std::bind(&ExperimentNode::result_callback, this, _1);

      auto descent_req = depth_descent_action->async_send_goal(
          descent_goal, send_goal_options);
      descent_req.wait();

      // Start the control loop timer
      timer_ = this->create_wall_timer(
          100ms, std::bind(&ExperimentNode::controlLoop, this));
    });

    gate_center_ = std::make_pair(960, 540); // default image center
    last_gate_detection_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "Initialization done. Waiting for detections...");
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

  // Helper to cycle to the next state (for the alignment cycle)
  MovementState getNextState(MovementState current) {
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
  void
  detectionsCallback(const auv_interfaces::msg::DetectionArray::SharedPtr msg) {
    bool found_gate = false;
    for (auto &detection : msg->detections) {
      if (detection.object == "Gate") {
        int center_x = (detection.x1 + detection.x2) / 2;
        int center_y = (detection.y1 + detection.y2) / 2;
        gate_center_ = std::make_pair(center_x, center_y);
        last_gate_detection_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Gate found at center: (%d, %d).",
                    center_x, center_y);
        found_gate = true;
        gate_found_ = true; // mark that gate is found
        break;              // process only the first gate detection
      }
    }
    if (!found_gate) {
      RCLCPP_INFO(this->get_logger(), "No Gate found in this DetectionArray.");
    }
  }

  // -------------------------------------------------------
  // Timer Callback: main control loop
  // -------------------------------------------------------
  void controlLoop() {
    // If the gate hasn't been detected at startup, perform yaw rotation in
    // place.
    if (!gate_found_) {
      geometry_msgs::msg::Twist cmd_vel_msg;
      // Set a fixed yaw rotation (e.g., 0.2 rad/s) and zero forward speed.
      cmd_vel_msg.angular.z = 0.2;
      cmd_vel_msg.linear.y = 0.0;
      RCLCPP_INFO(this->get_logger(), "Searching for gate: rotating in place.");
      cmd_vel_pub_->publish(cmd_vel_msg);
      return;
    }

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    // Compute error relative to the assumed image center (640x480 image)
    float image_center_x = 320.0f;
    float image_center_y = 240.0f;
    float error_x = static_cast<float>(gate_center_.first) - image_center_x;
    float error_y = static_cast<float>(gate_center_.second) - image_center_y;

    RCLCPP_INFO(this->get_logger(), "Gate error: x=%.1f, y=%.1f", error_x,
                error_y);
    RCLCPP_INFO(this->get_logger(), "Gate center: (%d, %d)", gate_center_.first,
                gate_center_.second);

    auto now_time = this->now();
    double elapsed_since_gate =
        (now_time - last_gate_detection_time_).seconds();

    if (elapsed_since_gate < gate_lost_threshold_ &&
        movement_state_ != LOST_GATE) {
      cmd_vel_msg = doAlignmentLogic();
    } else {
      if (movement_state_ != LOST_GATE) {
        movement_state_ = LOST_GATE;
        lost_gate_start_time_ = now_time;
        RCLCPP_WARN(this->get_logger(),
                    "[FSM] Gate is lost! Switching to LOST_GATE state.");
      }
      double elapsed_lost_state = (now_time - lost_gate_start_time_).seconds();
      if (elapsed_lost_state < forward_after_lost_) {
        cmd_vel_msg.linear.y = forward_speed_;
        cmd_vel_msg.angular.z = 0.0;
        RCLCPP_INFO(
            this->get_logger(),
            "[LOST_GATE] Moving forward for %.2f sec (elapsed=%.2f/%.2f)",
            forward_after_lost_, elapsed_lost_state, forward_after_lost_);
      } else if (elapsed_lost_state <
                 (forward_after_lost_ + pause_before_rotation_)) {
        RCLCPP_INFO(this->get_logger(),
                    "[LOST_GATE] Waiting for %.2f sec before rotating "
                    "(elapsed=%.2f/%.2f)",
                    pause_before_rotation_,
                    elapsed_lost_state - forward_after_lost_,
                    pause_before_rotation_);
      } else if (elapsed_lost_state <
                 (forward_after_lost_ + pause_before_rotation_ +
                  u_turn_duration_)) {
        cmd_vel_msg.angular.z = u_turn_yaw_speed_;
        RCLCPP_INFO(this->get_logger(),
                    "[LOST_GATE] Rotating for %.2f sec (elapsed=%.2f/%.2f), "
                    "speed=%.2f rad/s",
                    u_turn_duration_,
                    elapsed_lost_state - forward_after_lost_ -
                        pause_before_rotation_,
                    u_turn_duration_, u_turn_yaw_speed_);
      } else {
        cmd_vel_msg.angular.z = u_turn_yaw_speed_;
        RCLCPP_INFO(
            this->get_logger(),
            "[LOST_GATE] Continuing rotation until gate is reacquired.");
      }

      if (elapsed_since_gate < gate_lost_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "[LOST_GATE] Gate reacquired! Switching back to "
                    "HORIZONTAL_ALIGNMENT.");
        movement_state_ = HORIZONTAL_ALIGNMENT;
      }
    }
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // doAlignmentLogic: runs the horizontal->vertical->forward cyclical alignment
  // -------------------------------------------------------
  geometry_msgs::msg::Twist doAlignmentLogic() {
    geometry_msgs::msg::Twist cmd_vel_msg;
    float image_center_x = 320.0f;
    float image_center_y = 240.0f;
    float gate_x = static_cast<float>(gate_center_.first);
    float gate_y = static_cast<float>(gate_center_.second);
    float error_x = gate_x - image_center_x; // horizontal error
    float error_y = gate_y - image_center_y; // vertical error

    switch (movement_state_) {
    case HORIZONTAL_ALIGNMENT: {
      // Use a fixed yaw velocity (0.2 rad/s) based on the error direction.
      const double fixed_yaw_velocity = 0.2;
      if (error_x > 0) {
        cmd_vel_msg.angular.z = fixed_yaw_velocity;
      } else if (error_x < 0) {
        cmd_vel_msg.angular.z = -fixed_yaw_velocity;
      } else {
        cmd_vel_msg.angular.z = 0.0;
      }
      cmd_vel_msg.linear.y = 0.2;
      if (std::fabs(error_x) < x_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "[FSM] Horizontal alignment achieved. "
                                        "Switching to VERTICAL_ALIGNMENT.");
        movement_state_ = VERTICAL_ALIGNMENT;
      }
      break;
    }
    case VERTICAL_ALIGNMENT: {
      // No vertical correction (heave), maintain a small forward speed.
      cmd_vel_msg.linear.y = 0.2;
      cmd_vel_msg.linear.z = 0.0;
      if (std::fabs(error_y) < y_tolerance_) {
        RCLCPP_INFO(
            this->get_logger(),
            "[FSM] Vertical alignment acceptable. Switching to FORWARD_SURGE.");
        movement_state_ = FORWARD_SURGE;
      }
      break;
    }
    case FORWARD_SURGE: {
      // Send a fixed forward speed.
      cmd_vel_msg.linear.y = 0.5;
      cmd_vel_msg.angular.z = 0.0;
      if (std::fabs(error_x) > big_error_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "[FSM] Significant horizontal drift => switching back to "
                    "HORIZONTAL_ALIGNMENT.");
        movement_state_ = HORIZONTAL_ALIGNMENT;
      } else if (std::fabs(error_y) > big_error_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "[FSM] Significant vertical drift => switching back to "
                    "VERTICAL_ALIGNMENT.");
        movement_state_ = VERTICAL_ALIGNMENT;
      }
      break;
    }
    case LOST_GATE: {
      // This branch should not normally be reached in alignment logic.
      cmd_vel_msg.linear.y = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      break;
    }
    default:
      break;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "state=%d | error_x=%.1f, error_y=%.1f => forward=%.2f, yaw=%.4f",
        movement_state_, error_x, error_y, cmd_vel_msg.linear.y,
        cmd_vel_msg.angular.z);

    return cmd_vel_msg;
  }

  void goal_response_callback(const GoalHandleDepthDescent::SharedPtr &) {}

  void feedback_callback(GoalHandleDepthDescent::SharedPtr,
                         const std::shared_ptr<const DepthDescent::Feedback>) {}

  void result_callback(const GoalHandleDepthDescent::WrappedResult &) {}

  // -------------------------------------------------------
  // Member variables & parameters
  // -------------------------------------------------------
  rclcpp::Subscription<auv_interfaces::msg::DetectionArray>::SharedPtr
      detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr
      startup_timer_; // New timer for the initial delay
                      //
  rclcpp_action::Client<auv_interfaces::action::DepthDescent>::SharedPtr
      depth_descent_action;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr soft_arm_client_;
  // Gate center in image coordinates
  std::pair<int, int> gate_center_{320, 240};
  rclcpp::Time last_gate_detection_time_;
  rclcpp::Time lost_gate_start_time_;

  // FSM current state
  MovementState movement_state_{HORIZONTAL_ALIGNMENT};

  // Configurable parameters
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

  // Flag to indicate if the gate has been found at least once.
  bool gate_found_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperimentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
