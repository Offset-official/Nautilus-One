#include "auv_interfaces/action/read_comm_sequence.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <functional>
#include <memory>
#include <thread>

class ReadCommSequenceActionServer : public rclcpp::Node {
public:
  using ReadCommSequence = auv_interfaces::action::ReadCommSequence;
  using GoalHandleReadCommSequence =
      rclcpp_action::ServerGoalHandle<ReadCommSequence>;

  explicit ReadCommSequenceActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("read_comm_sequence_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ReadCommSequence>(
        this, "read_comm_sequence",
        std::bind(&ReadCommSequenceActionServer::handle_goal, this, _1),
        std::bind(&ReadCommSequenceActionServer::handle_cancel, this, _1),
        std::bind(&ReadCommSequenceActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ReadCommSequence>::SharedPtr action_server_;

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

  void execute(const std::shared_ptr<GoalHandleReadCommSequence> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ReadCommSequence::Feedback>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    auto result = std::make_shared<ReadCommSequence::Result>();

    for (int i = 1; (i < 3) && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      sequence.push_back(i);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ReadCommSequenceActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
