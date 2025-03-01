#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

class BagRecorder : public rclcpp::Node {
public:
    BagRecorder() : Node("bag_recorder"), is_recording_(false) {
        this->declare_parameter("bag_path", "recordings");
        
        // Create services for start/stop recording
        start_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_recording",
            std::bind(&BagRecorder::startRecording, this, 
                std::placeholders::_1, std::placeholders::_2));
        
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_recording",
            std::bind(&BagRecorder::stopRecording, this, 
                std::placeholders::_1, std::placeholders::_2));
    }

private:
    void startRecording(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
    {
        if (is_recording_) {
            response->success = false;
            response->message = "Already recording";
            return;
        }
    
        try {
            writer_ = std::make_unique<rosbag2_cpp::Writer>();
    
            auto now = std::chrono::system_clock::now();
            auto now_c = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << this->get_parameter("bag_path").as_string() << "/recording_";
            ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    
            writer_->open(ss.str());
    
            // Get all active topics and subscribe
            auto topic_names_and_types = this->get_topic_names_and_types();
            for (const auto& topic : topic_names_and_types) {
                const std::string& topic_name = topic.first;
                const std::string& topic_type = topic.second[0];
    
                RCLCPP_INFO(this->get_logger(), "Recording topic: %s", topic_name.c_str());
    
                rosbag2_storage::TopicMetadata topic_metadata;
                topic_metadata.name = topic_name;
                topic_metadata.type = topic_type;
                topic_metadata.serialization_format = "cdr";
                writer_->create_topic(topic_metadata);
    
                auto subscription = this->create_generic_subscription(
                    topic_name,
                    topic_type,
                    rclcpp::QoS(10),
                    [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                        if (!is_recording_ || !writer_) return;
                        
                        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                        bag_message->topic_name = topic_name;
                        bag_message->time_stamp = this->now().nanoseconds();
                        bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(msg->get_rcl_serialized_message());
                        
                        try {
                            writer_->write(bag_message);
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(this->get_logger(), "Failed to write message: %s", e.what());
                        }
                    }
                );
    
                subscriptions_.push_back(subscription);
            }
    
            is_recording_ = true;
            response->success = true;
            response->message = "Recording started";
            RCLCPP_INFO(this->get_logger(), "Started recording to %s", ss.str().c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to start recording: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
        }
    }
    
    void stopRecording(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!is_recording_) {
            response->success = false;
            response->message = "Not currently recording";
            return;
        }
    
        try {
            writer_.reset();
            subscriptions_.clear();
            is_recording_ = false;
            response->success = true;
            response->message = "Recording stopped";
            RCLCPP_INFO(this->get_logger(), "Stopped recording");
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Failed to stop recording: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Failed to stop recording: %s", e.what());
        }
    }

    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    bool is_recording_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BagRecorder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
