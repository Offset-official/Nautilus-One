#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "auv_interfaces/msg/detection_array.hpp"
#include "auv_interfaces/msg/detection.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include <chrono>
#include <vector>
#include <string>
#include <cctype>

// -----------------------------------------------------------------------------
// Base64 encoding/decoding helper functions
// (Simple implementations; you can replace these with your favorite library.)
static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

std::string base64_encode(const unsigned char* bytes_to_encode, unsigned int in_len) {
  std::string ret;
  int i = 0, j = 0;
  unsigned char char_array_3[3], char_array_4[4];

  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;
      for(i = 0; i < 4; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }
  if (i) {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';
    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;
    for (j = 0; j < i + 1; j++)
      ret += base64_chars[char_array_4[j]];
    while((i++ < 3))
      ret += '=';
  }
  return ret;
}

static inline bool is_base64(unsigned char c) {
  return (std::isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_decode(const std::string &encoded_string) {
  int in_len = encoded_string.size();
  int i = 0, j = 0, in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  std::string ret;

  while (in_len-- && ( encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
    char_array_4[i++] = encoded_string[in_]; in_++;
    if (i == 4) {
      for (i = 0; i < 4; i++)
        char_array_4[i] = base64_chars.find(char_array_4[i]);
      char_array_3[0] = ( char_array_4[0] << 2 ) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
      for (i = 0; i < 3; i++)
        ret += char_array_3[i];
      i = 0;
    }
  }
  if (i) {
    for (j = i; j < 4; j++)
      char_array_4[j] = 0;
    for (j = 0; j < 4; j++)
      char_array_4[j] = base64_chars.find(char_array_4[j]);
    char_array_3[0] = ( char_array_4[0] << 2 ) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];
    for (j = 0; j < i - 1; j++) ret += char_array_3[j];
  }
  return ret;
}

// -----------------------------------------------------------------------------
// CURL write callback function to capture response data
size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

using json = nlohmann::json;

class CameraInferenceNode : public rclcpp::Node
{
public:
  CameraInferenceNode() : Node("infer_camera_gate")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("camera_source_topic", "/auv_camera/image_raw");
    this->declare_parameter<std::string>("camera_pub_topic", "/auv_camera/image_inferred");
    this->declare_parameter<std::string>("detections_pub_topic", "/auv_camera/detections");
    this->declare_parameter<bool>("compressed", true);
    this->declare_parameter<std::string>("fastapi_server_ip", "192.168.2.4");

    // Get parameter values
    camera_source_topic_  = this->get_parameter("camera_source_topic").as_string();
    camera_pub_topic_     = this->get_parameter("camera_pub_topic").as_string();
    detections_pub_topic_ = this->get_parameter("detections_pub_topic").as_string();
    use_compressed_       = this->get_parameter("compressed").as_bool();
    fastapi_server_ip_    = this->get_parameter("fastapi_server_ip").as_string();
    fastapi_url_ = "http://" + fastapi_server_ip_ + ":8000/inference/gate";

    RCLCPP_INFO(get_logger(), "FastAPI URL: %s", fastapi_url_.c_str());

    setupSubscriptions();
    setupPublishers();
  }

private:
  // Parameters and member variables
  std::string camera_source_topic_;
  std::string camera_pub_topic_;
  std::string detections_pub_topic_;
  bool use_compressed_;
  std::string fastapi_server_ip_;
  std::string fastapi_url_;

  image_transport::Subscriber image_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<auv_interfaces::msg::DetectionArray>::SharedPtr detectionsPublisher_;

  // Setup image subscription using image_transport
  void setupSubscriptions()
  {
    auto callback = std::bind(&CameraInferenceNode::imageCallback, this, std::placeholders::_1);
    std::string transport_type = use_compressed_ ? "compressed" : "raw";
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    image_subscriber_ = image_transport::create_subscription(
      this, camera_source_topic_, callback, transport_type, qos.get_rmw_qos_profile());
  }
  

  // Setup publishers for inferred image and detection array
  void setupPublishers()
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(camera_pub_topic_, 10);
    detectionsPublisher_ = this->create_publisher<auv_interfaces::msg::DetectionArray>(detections_pub_topic_, 10);
  }

  // Callback for incoming image messages
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // Create a non-const shared pointer copy for processing
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>(*msg);
    processImage(image_msg);
  }

  // Process the image: perform FastAPI inference and publish results
  void processImage(const sensor_msgs::msg::Image::SharedPtr & img_msg)
  {
    auto start = std::chrono::steady_clock::now();
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat cv_image = cv_ptr->image;

    // Encode image to JPEG
    std::vector<uchar> buf;
    if (!cv::imencode(".jpg", cv_image, buf)) {
      RCLCPP_ERROR(get_logger(), "Failed to encode image to JPEG");
      return;
    }

    // Base64 encode the JPEG data
    std::string img_base64 = base64_encode(buf.data(), buf.size());

    // Create JSON payload
    json payload;
    payload["image"] = img_base64;
    std::string payload_str = payload.dump();

    // Initialize CURL for HTTP POST
    CURL *curl = curl_easy_init();
    if (!curl) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize CURL");
      return;
    }

    CURLcode res;
    std::string response_string;
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, fastapi_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload_str.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
      RCLCPP_ERROR(get_logger(), "CURL request failed: %s", curl_easy_strerror(res));
      curl_easy_cleanup(curl);
      curl_slist_free_all(headers);
      return;
    }

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    if (http_code != 200) {
      RCLCPP_ERROR(get_logger(), "FastAPI server returned HTTP code %ld, response: %s", http_code, response_string.c_str());
      curl_easy_cleanup(curl);
      curl_slist_free_all(headers);
      return;
    }
    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);

    // Parse JSON response
    json result;
    try {
      result = json::parse(response_string);
    } catch (json::parse_error &e) {
      RCLCPP_ERROR(get_logger(), "JSON parse error: %s", e.what());
      return;
    }

    // Retrieve and decode the annotated image from the response
    if (!result.contains("annotated_image")) {
      RCLCPP_ERROR(get_logger(), "Response does not contain 'annotated_image'");
      return;
    }
    std::string annotated_image_b64 = result["annotated_image"];
    std::string annotated_image_bytes_str = base64_decode(annotated_image_b64);
    std::vector<uchar> annotated_buf(annotated_image_bytes_str.begin(), annotated_image_bytes_str.end());
    cv::Mat annotated_image = cv::imdecode(annotated_buf, cv::IMREAD_COLOR);
    if (annotated_image.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to decode annotated image");
      return;
    }

    // Process detections from the JSON response
    auv_interfaces::msg::DetectionArray detectionArrayMsg;
    if (result.contains("detections") && result["detections"].is_array()) {
      for (const auto& det : result["detections"]) {
        auv_interfaces::msg::Detection detection_msg;
        detection_msg.object = det.value("object", "");
        detection_msg.confidence = det.value("confidence", 0.0);
        detection_msg.x1 = det.value("x1", 0);
        detection_msg.y1 = det.value("y1", 0);
        detection_msg.x2 = det.value("x2", 0);
        detection_msg.y2 = det.value("y2", 0);
        detectionArrayMsg.detections.push_back(detection_msg);
      }
    }

    // Convert the annotated image back to a ROS Image message
    cv_bridge::CvImage out_msg;
    out_msg.header = img_msg->header;
    out_msg.encoding = "bgr8";
    out_msg.image = annotated_image;
    auto result_image_msg = out_msg.toImageMsg();

    // Publish the results
    publisher_->publish(*result_image_msg);
    detectionsPublisher_->publish(detectionArrayMsg);

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(get_logger(), "Inference Time: %.3f sec, Detections: %lu", elapsed.count(), detectionArrayMsg.detections.size());
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraInferenceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
