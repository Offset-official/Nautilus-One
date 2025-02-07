#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// For publishing velocity commands
#include <geometry_msgs/msg/twist.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

// C++ includes
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

    // --- Declare parameter and set up callback for changes ---
    this->declare_parameter<int>("follow_id", 0);
    follow_id_ = this->get_parameter("follow_id").as_int();

    // If your ROS 2 version supports it, you can capture this handle for later use
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

            // Reset the last_april_tag_time_ so we won't move until this new ID is detected
            last_april_tag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

            RCLCPP_INFO(
              this->get_logger(),
              "Parameter 'follow_id' changed to %d. Waiting for that tag to appear...",
              follow_id_);
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

    // 3) Timer for control loop (runs every 0.1 seconds)
    timer_ = this->create_wall_timer(
      1000ms,
      std::bind(&AprilFollowNode::controlLoop, this)
    );

    // Initialize last_april_tag_time_ to zero => haven't seen any tag yet
    last_april_tag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    RCLCPP_INFO(this->get_logger(),
                "Initialization done. Will follow AprilTag ID = %d once detected...",
                follow_id_);
  }

private:
  // -------------------------------------------------------
  // Callback: receives AprilTagDetectionArray messages
  // -------------------------------------------------------
  // FIX: Removed the extra '>'
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    bool found_tag = false;

    for (auto &detection : msg->detections)
    {
      // Check if this detection has the ID we care about
      if (detection.id == follow_id_)
      {
        // Retrieve center of the AprilTag in image coords
        double center_x = detection.centre.x;  // centre.x is float64
        double center_y = detection.centre.y;

        // Update our stored tag center
        april_center_ = std::make_pair(
          static_cast<int>(center_x),
          static_cast<int>(center_y)
        );

        // Update the time we last saw the AprilTag
        last_april_tag_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "AprilTag ID=%d found at center: (%.2f, %.2f).",
                    follow_id_, center_x, center_y);

        found_tag = true;
        break; // Stop after finding the first matching ID
      }
    }

    if (!found_tag)
    {
      RCLCPP_DEBUG(this->get_logger(),
                   "No AprilTag with ID=%d found in this detection array.",
                   follow_id_);
    }
  }

  // -------------------------------------------------------
  // Timer Callback: main control loop
  // -------------------------------------------------------
  void controlLoop()
  {
    // Default: stop
    geometry_msgs::msg::Twist cmd_vel_msg;

    // If we've never seen a tag since startup/param-change, remain still
    if (last_april_tag_time_.nanoseconds() == 0)
    {
      cmd_vel_pub_->publish(cmd_vel_msg);
      return;
    }

    // Calculate how long since we last saw the AprilTag
    double elapsed_since_tag = (this->now() - last_april_tag_time_).seconds();

    // If we've seen the tag within the past 2s, align with it
    const double TAG_LOST_THRESHOLD = 2.0;
    if (elapsed_since_tag < TAG_LOST_THRESHOLD)
    {
      cmd_vel_msg = doAprilTagAlignment();
    }
    else
    {
      // If tag is lost for >= 2s, stop all motion
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000, // once every 2 seconds
        "AprilTag ID=%d lost for %.2f seconds => Stopping robot.", follow_id_, elapsed_since_tag);
    }

    // Publish command
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  // -------------------------------------------------------
  // doAprilTagAlignment:
  //   tries to align the robot so that the AprilTag is near
  //   the center of the image
  // -------------------------------------------------------
  geometry_msgs::msg::Twist doAprilTagAlignment()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;

    // Assume a typical image size of 640 x 480
    float image_center_x = 320.0f;
    float image_center_y = 240.0f;

    float tag_x = static_cast<float>(april_center_.first);
    float tag_y = static_cast<float>(april_center_.second);

    // Errors in image coordinates:
    float error_x = tag_x - image_center_x; // horizontal offset
    float error_y = tag_y - image_center_y; // vertical offset

    // Gains (tune these as needed)
    float kp_yaw   = 0.0005f;  // horizontal alignment (yaw)
    float kp_heave = -0.005f;  // vertical alignment (z)

    // Generate commands
    float yaw_cmd = kp_yaw * error_x;
    float z_cmd   = kp_heave * error_y;

    // Mild forward speed to move closer
    float forward_speed = 0.2f;

    cmd_vel_msg.linear.x  = forward_speed;
    cmd_vel_msg.linear.z  = z_cmd;
    cmd_vel_msg.angular.z = yaw_cmd;

    RCLCPP_INFO(this->get_logger(),
                "Tag alignment => err_x=%.1f err_y=%.1f => fwd=%.2f, z=%.2f, yaw=%.4f",
                error_x, error_y,
                cmd_vel_msg.linear.x, cmd_vel_msg.linear.z, cmd_vel_msg.angular.z);

    return cmd_vel_msg;
  }

  // -------------------------------------------------------
  // Member variables
  // -------------------------------------------------------
  // Update to Node::OnSetParametersCallbackHandle::SharedPtr
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // The ID of the AprilTag to follow
  int follow_id_{0};

  // Track the AprilTag center from detections (in image coords)
  std::pair<int,int> april_center_{320, 240};

  // Track when we last saw the AprilTag.
  // If zero, the robot will not move until it detects the tag the first time.
  rclcpp::Time last_april_tag_time_;
};

// -------------------------------------------------------
// main: create and spin the node
// -------------------------------------------------------
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AprilFollowNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
