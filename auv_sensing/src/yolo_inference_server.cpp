#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

// Custom service and message headers generated from .srv and .msg files
#include "auv_sensing/srv/yolo_inference.hpp"
#include "auv_sensing/msg/detection.hpp"

// OpenCV
#include <opencv2/opencv.hpp>

// ONNX Runtime C++ API
#include <onnxruntime_cxx_api.h>

// STL
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

// -----------------------------------------------------------
// YOLOv8 class: Loads an ONNX model, runs inference, draws detections.
// -----------------------------------------------------------
class YoloV8
{
public:
  YoloV8(const std::string &model_path,
         float conf_thres,
         float iou_thres,
         const std::vector<std::string> &class_names,
         rclcpp::Logger logger)
  : confidence_threshold_(conf_thres),
    iou_threshold_(iou_thres),
    classes_(class_names),
    logger_(logger)
  {
    // Initialize random color palette for each class
    generateColorPalette();

    // Initialize ONNX Runtime session
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "yolo_inference");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);

    // (Optional) enable/disable optimizations
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Create the session
    try
    {
      session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
    }
    catch (const Ort::Exception &e)
    {
      RCLCPP_ERROR(logger_, "Failed to create ONNX Runtime session: %s", e.what());
      throw;
    }

    // Get input details
    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputName(0, allocator);
    RCLCPP_INFO(logger_, "Model input name: %s", input_name_.c_str());

    // Example for a fixed-size model: (batch, channels, height, width)
    auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    // Typically: input_shape = [1, 3, 640, 640] or something similar

    if (input_shape.size() != 4)
    {
      RCLCPP_ERROR(logger_, "Unexpected input shape size: %zu", input_shape.size());
      throw std::runtime_error("Unsupported input shape size");
    }

    input_height_ = static_cast<int>(input_shape[2]);
    input_width_  = static_cast<int>(input_shape[3]);

    RCLCPP_INFO(logger_, "Model input dimensions: %d x %d", input_height_, input_width_);
  }

  // ---------------------------------------------------------
  // Run full inference (preprocess -> forward -> postprocess).
  // ---------------------------------------------------------
  std::pair<cv::Mat, std::vector<auv_sensing::msg::Detection>> runInference(const cv::Mat &bgr_image)
  {
    // Copy image for annotation
    cv::Mat annotated = bgr_image.clone();

    // 1. Preprocess
    int orig_width  = bgr_image.cols;
    int orig_height = bgr_image.rows;
    Ort::Value input_tensor = preprocess(bgr_image);

    // 2. Inference
    std::array<const char *, 1> input_names = { input_name_.c_str() };
    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(std::move(input_tensor));

    std::vector<const char*> output_names; // nullptr to get all outputs
    // Optionally, retrieve output names if needed

    std::vector<Ort::Value> output_tensors;
    try
    {
      output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names.data(),
        input_tensors.data(),
        input_tensors.size(),
        nullptr,    // output names (nullptr => all outputs)
        0           // output count (0 => all)
      );
    }
    catch (const Ort::Exception &e)
    {
      RCLCPP_ERROR(logger_, "ONNX Runtime inference failed: %s", e.what());
      throw;
    }

    if (output_tensors.empty())
    {
      RCLCPP_ERROR(logger_, "No output tensors received from inference.");
      throw std::runtime_error("No output tensors received from inference.");
    }

    // The first (and possibly only) output is YOLO head
    Ort::Value &raw_output = output_tensors[0];
    // The shape might be [1, X, 85] or [1, 85, X] depending on your model
    // In the example, we flatten or interpret as needed in postprocess.

    // 3. Postprocess (parse boxes, apply NMS, draw results)
    float *output_data = raw_output.GetTensorMutableData<float>();
    // You may need to query the output shape to handle it properly
    std::vector<int64_t> out_shape = 
      raw_output.GetTensorTypeAndShapeInfo().GetShape();
    // out_shape might be [1, num_values, 85], etc.

    std::vector<auv_sensing::msg::Detection> detections;
    postprocess(annotated, output_data, out_shape, orig_width, orig_height, detections);

    return { annotated, detections };
  }

private:
  // Preprocess: BGR->RGB, resize, normalize, CHW, float32
  Ort::Value preprocess(const cv::Mat &bgr_image)
  {
    // Resize to model dims
    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(input_width_, input_height_));

    // BGR -> RGB
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

    // Convert to float, normalize [0,1]
    resized.convertTo(resized, CV_32F, 1.0f / 255.0f);

    // HWC -> CHW
    std::vector<float> input_tensor_values;
    input_tensor_values.resize(input_width_ * input_height_ * 3);
    // Layout: (3, input_height_, input_width_)
    int index = 0;
    for (int c = 0; c < 3; c++)
    {
      for (int y = 0; y < input_height_; y++)
      {
        for (int x = 0; x < input_width_; x++)
        {
          input_tensor_values[index++] = resized.at<cv::Vec3f>(y, x)[c];
        }
      }
    }

    // Create ONNX Tensor
    std::array<int64_t, 4> input_shape = {1, 3, input_height_, input_width_};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info,
      input_tensor_values.data(),
      input_tensor_values.size(),
      input_shape.data(),
      input_shape.size()
    );

    return input_tensor;
  }

  // Postprocess: parse boxes, filter by confidence, apply NMS, draw on annotated image.
  // Fills out 'detections' with Detection messages.
  void postprocess(cv::Mat &annotated,
                   float *output_data,
                   const std::vector<int64_t> &output_shape,
                   int orig_width,
                   int orig_height,
                   std::vector<auv_sensing::msg::Detection> &detections)
  {
    // Example YOLO output shape: [1, n, 85] => each row: [x, y, w, h, conf, p1, p2, ...]
    // Possibly: x,y,w,h are center-based coords, or left-top-based, etc.
    // This code is reference only; adapt to your actual output format.
    if (output_shape.size() < 3)
    {
      RCLCPP_ERROR(logger_, "Unexpected output shape size: %zu", output_shape.size());
      return;
    }

    int64_t num_boxes = output_shape[1];
    int64_t num_attrs = output_shape[2]; // 85 for YOLOv8 (x,y,w,h,conf + 80 classes)

    // Gather raw boxes + scores
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    for (int64_t i = 0; i < num_boxes; i++)
    {
      float x_center = output_data[i * num_attrs + 0];
      float y_center = output_data[i * num_attrs + 1];
      float w        = output_data[i * num_attrs + 2];
      float h        = output_data[i * num_attrs + 3];
      float conf     = output_data[i * num_attrs + 4];

      // Find the class with max probability
      float max_class_prob = -1.0f;
      int max_class_id = -1;
      for (int c = 5; c < static_cast<int>(num_attrs); c++)
      {
        float class_prob = output_data[i * num_attrs + c];
        if (class_prob > max_class_prob)
        {
          max_class_prob = class_prob;
          max_class_id = c - 5;
        }
      }

      float score = conf * max_class_prob;
      if (score > confidence_threshold_)
      {
        // Convert from center-based coords to top-left
        int left = static_cast<int>((x_center - 0.5f * w) * static_cast<float>(orig_width));
        int top  = static_cast<int>((y_center - 0.5f * h) * static_cast<float>(orig_height));
        int width = static_cast<int>(w * static_cast<float>(orig_width));
        int height= static_cast<int>(h * static_cast<float>(orig_height));

        boxes.emplace_back(left, top, width, height);
        confidences.push_back(score);
        class_ids.push_back(max_class_id);
      }
    }

    // Apply NMS
    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, iou_threshold_, nms_indices);

    // Build detection results + draw
    for (int idx : nms_indices)
    {
      if (idx >= static_cast<int>(boxes.size()))
      {
        RCLCPP_WARN(logger_, "NMS index out of range: %d", idx);
        continue;
      }

      auto box = boxes[idx];
      float sc = confidences[idx];
      int cls  = class_ids[idx];

      drawDetection(annotated, box, sc, cls);

      // Populate Detection message
      auv_sensing::msg::Detection det;
      det.object = classes_[cls];
      det.confidence = sc;
      det.x1 = box.x;
      det.y1 = box.y;
      det.x2 = box.x + box.width;
      det.y2 = box.y + box.height;

      // Add to detections vector
      detections.push_back(det);
    }
  }

  // Draw bounding box + label on the image
  void drawDetection(cv::Mat &img, const cv::Rect &box, float score, int class_id)
  {
    cv::Scalar color = colors_[class_id % colors_.size()];
    cv::rectangle(img, box, color, 2);

    std::string label = classes_[class_id] + ": " + std::to_string(score);

    int baseLine = 0;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    int top = std::max(box.y, labelSize.height);
    cv::rectangle(img, 
                  cv::Point(box.x, top - labelSize.height),
                  cv::Point(box.x + labelSize.width, top + baseLine),
                  color, cv::FILLED);
    cv::putText(img, label, cv::Point(box.x, top),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
  }

  // Generate a random color palette for each class
  void generateColorPalette()
  {
    srand(static_cast<unsigned int>(time(nullptr)));
    for (size_t i = 0; i < classes_.size(); i++)
    {
      colors_.push_back(cv::Scalar(rand() % 256, rand() % 256, rand() % 256));
    }
  }

private:
  float confidence_threshold_;
  float iou_threshold_;

  int input_width_  = 640;  // modified automatically
  int input_height_ = 640;  // modified automatically

  std::vector<std::string> classes_;
  std::vector<cv::Scalar> colors_;

  // ONNX Runtime
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;

  // Logger
  rclcpp::Logger logger_;
};

// -----------------------------------------------------------
// Service Node: Advertises the /yolo_inference service
// -----------------------------------------------------------
class YoloInferenceServer : public rclcpp::Node
{
public:
  YoloInferenceServer()
  : Node("yolo_inference_server")
  {
    // Create the YOLO model instance
    // (Adjust path, thresholds, class names, etc. as needed)
    std::string model_path = "best.onnx";  // Ensure this path is correct
    float conf_thres = 0.5f;
    float iou_thres  = 0.5f;
    std::vector<std::string> class_names = {
      "Blue flare", "Blue pail", "Cloth", "Gate",
      "Red flare", "Red pail", "Yellow flare"
    };

    try
    {
      yolo_ = std::make_shared<YoloV8>(model_path, conf_thres, iou_thres, class_names, this->get_logger());
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize YoloV8: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // Create the service
    using ServiceT = auv_sensing::srv::YoloInference;
    service_ = this->create_service<ServiceT>(
        "yolo_inference",
        std::bind(&YoloInferenceServer::handleInference, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "YoloInferenceService is ready.");
  }

private:
  // Service callback
  void handleInference(
      const std::shared_ptr<auv_sensing::srv::YoloInference::Request> request,
      std::shared_ptr<auv_sensing::srv::YoloInference::Response> response)
  {
    // Convert ROS Image -> cv::Mat using toCvCopy
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(request->image, "bgr8");
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Run inference
    std::pair<cv::Mat, std::vector<auv_sensing::msg::Detection>> result;
    try
    {
      result = yolo_->runInference(cv_ptr->image);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Inference failed: %s", e.what());
      return;
    }

    cv::Mat annotated_image = result.first;
    std::vector<auv_sensing::msg::Detection> detections = result.second;

    // Convert annotated cv::Mat -> ROS Image
    cv_bridge::CvImage out_msg;
    out_msg.encoding = "bgr8";
    out_msg.image = annotated_image;
    response->result_image = *out_msg.toImageMsg();

    // Fill detections
    response->detections = detections;

    RCLCPP_INFO(this->get_logger(), "Inference completed with %zu detections.", detections.size());
  }

private:
  rclcpp::Service<auv_sensing::srv::YoloInference>::SharedPtr service_;
  std::shared_ptr<YoloV8> yolo_;
};

// -----------------------------------------------------------
// main()
// -----------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YoloInferenceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
