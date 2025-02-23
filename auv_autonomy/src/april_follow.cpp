#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <vector>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class AprilFollowNode : public rclcpp::Node
{
public:
  AprilFollowNode()
  : Node("april_follow_node")
  {
    RCLCPP_INFO(this->get_logger(), "AprilFollowNode has started.");

    // --- Declare parameters and retrieve them ---
    this->declare_parameter<int>("follow_id", 0);
    follow_id_ = this->get_parameter("follow_id").as_int();

    this->declare_parameter<double>("forward_speed", 0.3);
    forward_speed_ = this->get_parameter("forward_speed").as_double();

    this->declare_parameter<double>("kp_yaw", 0.01);
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();

    // Parameter-change callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (auto &param : params)
        {
          if (param.get_name() == "follow_id")
          {
            follow_id_ = param.as_int();
            // Reset so we won't move until we see the new ID
            last_april_tag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            // Also reset to WAITING_FOR_TAG
            state_ = FollowState::WAITING_FOR_TAG;

            RCLCPP_INFO(
              this->get_logger(),
              "Parameter 'follow_id' changed to %d. Waiting for that tag to appear...",
              follow_id_);
          }
          else if (param.get_name() == "forward_speed")
          {
            forward_speed_ = param.as_double();
            RCLCPP_INFO(
              this->get_logger(),
              "Parameter 'forward_speed' changed to %.3f.",
              forward_speed_);
          }
          else if (param.get_name() == "kp_yaw")
          {
            kp_yaw_ = param.as_double();
            RCLCPP_INFO(
              this->get_logger(),
              "Parameter 'kp_yaw' changed to %.3f.",
              kp_yaw_);
          }
        }
        return result;
      }
    );

    // 1) Subscribe to /detections (AprilTagDetectionArray)
    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/detections",
      10,
      std::bind(&AprilFollowNode::detectionsCallback, this, std::placeholders::_1)
    );

    // 2) Publisher for /cmd_vel
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 3) Timer for control loop (every 400 ms)
    timer_ = this->create_wall_timer(
      400ms,
      std::bind(&AprilFollowNode::controlLoop, this)
    );

    // Initially, we haven't seen the tag => WAITING_FOR_TAG
    state_ = FollowState::WAITING_FOR_TAG;
    // Set time to zero => not seen yet
    last_april_tag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    RCLCPP_INFO(this->get_logger(),
                "Initialization done. Will follow AprilTag ID = %d once detected...",
                follow_id_);
  }

  // Destructor: publish zero velocity on shutdown
  ~AprilFollowNode() override
  {
    // Publish a single stop command
    RCLCPP_INFO(this->get_logger(),
                "AprilFollowNode shutting down...publishing zero velocity.");
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);

    // (Optional) Sleep briefly to ensure message is sent
    // rclcpp::sleep_for(200ms);
  }

private:
  // -------------------------------------------------------
  // Finite State Machine states
  // -------------------------------------------------------
  enum class FollowState
  {
    WAITING_FOR_TAG,
    FOLLOWING_TAG
  };

  // -------------------------------------------------------
  // Callback: receives AprilTagDetectionArray messages
  // -------------------------------------------------------
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    bool found_tag = false;

    for (auto &detection : msg->detections)
    {
      // Check if this detection has the ID we care about
      if (detection.id == follow_id_)
      {
        // Retrieve center in image coords
        double center_x = detection.centre.x;
        double center_y = detection.centre.y;

        // Update stored tag center
        april_center_ = std::make_pair(
          static_cast<int>(center_x),
          static_cast<int>(center_y)
        );

        // Update the time we last saw this tag
        last_april_tag_time_ = this->now();
        found_tag = true;
        break; // We only need the first matching ID
      }
    }

    // If we found the tag, switch to FOLLOWING_TAG (if not already)
    if (found_tag && state_ != FollowState::FOLLOWING_TAG)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Found AprilTag ID=%d => switching to FOLLOWING_TAG.", follow_id_);
      state_ = FollowState::FOLLOWING_TAG;
    }
  }

  // -------------------------------------------------------
  // Timer Callback: main control loop
  // -------------------------------------------------------
  void controlLoop()
  {
    // Default is zero velocity
    geometry_msgs::msg::Twist cmd_vel_msg;

    // We use a simple 2-state FSM:
    switch (state_)
    {
      case FollowState::WAITING_FOR_TAG:
      {
        // Do nothing (stop). We'll switch to FOLLOWING_TAG as soon as
        // detectionsCallback sees the desired ID.
        break;
      }

      case FollowState::FOLLOWING_TAG:
      {
        // Check how long since last tag detection
        double elapsed_since_tag = (this->now() - last_april_tag_time_).seconds();

        // If we haven't seen the tag for 2 seconds, go back to WAITING_FOR_TAG
        const double TAG_LOST_THRESHOLD = 2.0;
        if (elapsed_since_tag >= TAG_LOST_THRESHOLD)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Lost AprilTag ID=%d for %.2f s => switching to WAITING_FOR_TAG.",
                      follow_id_, elapsed_since_tag);
          state_ = FollowState::WAITING_FOR_TAG;
        }
        else
        {
          // Tag is still "fresh"; align with it
          cmd_vel_msg = doAprilTagAlignment();
        }
        break;
      }
    }

    // Publish velocity command
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // doAprilTagAlignment:
  //   tries to align robot yaw so the AprilTag is near
  //   the image center, and moves forward at a set speed.
  // -------------------------------------------------------
  geometry_msgs::msg::Twist doAprilTagAlignment()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;

    // Assume typical image size of 640 x 480
    float image_center_x = 320.0f;
    // float image_center_y = 240.0f; // Not used for vertical correction

    float tag_x = static_cast<float>(april_center_.first);

    // Horizontal offset
    float error_x = tag_x - image_center_x;

    // Yaw command from param kp_yaw_
    float yaw_cmd = kp_yaw_ * error_x;

    // Forward speed from param
    cmd_vel_msg.linear.y  = forward_speed_;
    cmd_vel_msg.angular.z = yaw_cmd;

    RCLCPP_INFO(this->get_logger(),
                "->Tag alignment => err_x=%.1f => forward=%.2f, yaw=%.4f",
                error_x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);

    return cmd_vel_msg;
  }

  // -------------------------------------------------------
  // Member variables
  // -------------------------------------------------------
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // FSM state
  FollowState state_{FollowState::WAITING_FOR_TAG};

  // The ID of the AprilTag to follow
  int follow_id_{0};

  // Gains and speeds (loaded from parameters)
  double forward_speed_{0.3};
  double kp_yaw_{0.01};

  // Last-known center of the AprilTag in image coords
  std::pair<int,int> april_center_{320, 240};

  // Track when we last saw the target AprilTag
  rclcpp::Time last_april_tag_time_;
};

// -------------------------------------------------------
// main: create and spin the node
// -------------------------------------------------------
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // Create the node inside a shared pointer
  auto node = std::make_shared<AprilFollowNode>();

  // Spin until Ctrl + C (or node shutdown). 
  // When Ctrl + C is pressed, rclcpp::shutdown() will be invoked,
  // which triggers our node's destructor to publish zero velocity.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
