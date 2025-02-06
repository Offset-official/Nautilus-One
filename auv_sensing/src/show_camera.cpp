#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ShowCameraNode : public rclcpp::Node 
{
    public:
        ShowCameraNode(const char* camera_topic) : Node("show_camera") 
        {
            RCLCPP_INFO(this->get_logger(), "Starting node...");
            
            auto topics = this->get_topic_names_and_types();
            // bool topic_exists = topics.find(camera_topic) != topics.end();
            // if (topic_exists) {
            //     RCLCPP_INFO(this->get_logger(), "Topic %s is alive", camera_topic);
            //     RCLCPP_INFO(this->get_logger(), "Start Gazebo to see the camera feed.");
            // }
            // else {
            //     RCLCPP_ERROR(this->get_logger(), "Topic %s is not alive. Exiting...", camera_topic);
            //     rclcpp::shutdown();
            // }
            
            try {
                subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                    camera_topic, 10, std::bind(&ShowCameraNode::image_callback, this, std::placeholders::_1)
                                );
            }
            catch (rclcpp::exceptions::RCLError& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
                rclcpp::shutdown();
            }
        }
    private:
 
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const {
            try {
                // RCLCPP_INFO(this->get_logger(), "Image Received");
                cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
                cv::imshow("Camera", frame);
                cv::waitKey(10);
            }
            catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                rclcpp::shutdown();
            }

        }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
 
};

int main(int argc, char* argv[]) {
    const char* camera_topic = argv[1];
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShowCameraNode>(camera_topic));
    rclcpp::shutdown();
    return 0;

}