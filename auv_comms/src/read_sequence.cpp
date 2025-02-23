#include "auv_interfaces/action/read_comm_sequence.hpp"
#include "auv_interfaces/srv/image_color_detect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <chrono>
#include <functional>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;

class ReadCommSequenceActionServer : public rclcpp::Node {
public:
  using ReadCommSequence = auv_interfaces::action::ReadCommSequence;
  using GoalHandleReadCommSequence =
      rclcpp_action::ServerGoalHandle<ReadCommSequence>;
  using ImageColorDetect = auv_interfaces::srv::ImageColorDetect;

  explicit ReadCommSequenceActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("read_comm_sequence_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ReadCommSequence>(
        this, "read_comm_sequence",
        std::bind(&ReadCommSequenceActionServer::handle_goal, this, _1),
        std::bind(&ReadCommSequenceActionServer::handle_cancel, this, _1),
        std::bind(&ReadCommSequenceActionServer::handle_accepted, this, _1));

    this->color_detect_client_ =
        this->create_client<ImageColorDetect>("/detect_color");

    this->image_sub_ = image_transport::create_subscription(
        this, "input_image",
        std::bind(&ReadCommSequenceActionServer::image_callback, this, _1),
        "compressed", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    while (!color_detect_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "/detect_color service not available, waiting again...");
    }

    rclcpp::Rate loop_rate(1s);
    while (image_sub_.getNumPublishers() < 1) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Interrupted while waiting for the camera publisher. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Not recieveing images, waiting again...");
      loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "Ready");
  }

private:
  rclcpp_action::Server<ReadCommSequence>::SharedPtr action_server_;
  rclcpp::Client<ImageColorDetect>::SharedPtr color_detect_client_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::msg::Image::ConstSharedPtr img_;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    img_ = msg;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleReadCommSequence> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleReadCommSequence> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&ReadCommSequenceActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  int compute_idx_from_color_str(std::string input) {
    if (input == "red") {
      return 0; // red flare
    }
    if (input == "green") {
      return 1; // yello flare
    }
    if (input == "blue") {
      return 2; // blue flare
    }
    return -1; // none detected
  }

  void execute(const std::shared_ptr<GoalHandleReadCommSequence> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ReadCommSequence::Feedback>();
    auto &sequence = feedback->partial_sequence;
    auto result = std::make_shared<ReadCommSequence::Result>();

    auto request =
        std::make_shared<auv_interfaces::srv::ImageColorDetect::Request>();
    if (!img_) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action Server Node did not recieve any images");
      result->sequence = sequence;
      goal_handle->abort(result);
      return;
    }

    request->image.header = img_->header;
    request->image.height = img_->height;
    request->image.width = img_->width;
    request->image.step = img_->step;
    request->image.encoding = img_->encoding;
    request->image.data = img_->data;

    while (sequence.size() != 3) {

      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      auto _result = color_detect_client_->async_send_request(request);
      _result.wait();
      auto detected_color = std::string(_result.get()->color.c_str());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detected: %s",
                  detected_color.c_str());
      auto idx = compute_idx_from_color_str(detected_color);

      if ((idx != -1) && (sequence.size() == 0 || idx != sequence.back())) {
        sequence.push_back(idx);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");
      }
      loop_rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Finished reading all signals");

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ReadCommSequenceActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
