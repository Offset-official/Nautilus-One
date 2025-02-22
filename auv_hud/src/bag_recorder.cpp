#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <iomanip>

class BagRecorder : public rclcpp::Node {
public:
    BagRecorder() : Node("bag_recorder"), is_recording_(false) {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("auv_hud");

        // Declare parameters with default values
        this->declare_parameter("config_file", package_share_dir + "/config/topics.yaml");
        this->declare_parameter("bag_path", "recordings");

        RCLCPP_INFO(this->get_logger(), "Config file path: %s", 
            this->get_parameter("config_file").as_string().c_str());
        
        // Initialize the recorder
        initialize();
        
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
    void initialize() {
        std::string config_file = this->get_parameter("config_file").as_string();
        loadTopicsFromYaml(config_file);
        
        for (const auto& topic : topics_) {
            RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic.name.c_str());
            
            auto subscription = this->create_generic_subscription(
                topic.name,
                topic.type,
                rclcpp::QoS(10),
                [this, topic_name = topic.name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                    if (!is_recording_ || !writer_) return;
                    
                    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                    bag_message->topic_name = topic_name;
                    bag_message->time_stamp = this->now().nanoseconds();
                    
                    // Get the raw message
                    auto serialized_msg = msg->get_rcl_serialized_message();
                    
                    // Create a new array and copy the data
                    auto data = std::make_shared<rcutils_uint8_array_t>();
                    data->buffer = new uint8_t[serialized_msg.buffer_length];
                    data->buffer_length = serialized_msg.buffer_length;
                    data->buffer_capacity = serialized_msg.buffer_length;
                    std::memcpy(data->buffer, serialized_msg.buffer, serialized_msg.buffer_length);
                    
                    bag_message->serialized_data = data;
                    
                    try {
                        writer_->write(bag_message);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to write message: %s", e.what());
                    }
                }
            );
            
            subscriptions_.push_back(subscription);
        }
    }

    void loadTopicsFromYaml(const std::string& config_file) {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            for (const auto& topic : config["topics"]) {
                TopicInfo info;
                info.name = topic["name"].as<std::string>();
                info.type = topic["type"].as<std::string>();
                topics_.push_back(info);
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
            throw;
        }
    }

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
            ss << this->get_parameter("bag_path").as_string() << "/";
            ss << "recording_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
    
            writer_->open(ss.str());
    
            // Register topics before writing to them
            for (const auto& topic : topics_) {
                rosbag2_storage::TopicMetadata topic_metadata;
                topic_metadata.name = topic.name;
                topic_metadata.type = topic.type;
                topic_metadata.serialization_format = "cdr"; 
    
                writer_->create_topic(topic_metadata);
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

    struct TopicInfo {
        std::string name;
        std::string type;
    };

    std::vector<TopicInfo> topics_;
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