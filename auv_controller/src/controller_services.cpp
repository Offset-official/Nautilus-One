#include "auv_controller/dumb_controller.hpp"

void DumbController::arm_vehicle(bool arm) {
  while (!arm_client_->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(),
                "Waiting for arm service to be available...");
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = arm;

  bool command_success = false;
  while (!command_success && rclcpp::ok()) {
    auto future = arm_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      if (future.get()->success) {
        command_success = true;
        RCLCPP_INFO(this->get_logger(), "Vehicle %s successfully",
                    arm ? "armed" : "disarmed");
      } else {
        RCLCPP_WARN(this->get_logger(), "Vehicle %s command received but not successful, retrying...",
                    arm ? "arm" : "disarm");
        // Add a small delay before retrying to avoid spamming
        std::this_thread::sleep_for(500ms);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call arm service, retrying...");
      // Add a small delay before retrying
      std::this_thread::sleep_for(500ms);
    }
  }
}

void DumbController::soft_arm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (request->data) {
    send_calibration_request();
    soft_arm_ = true;
    auto color_request =
        std::make_shared<auv_interfaces::srv::SetColor::Request>();
    color_request->color = "#AFE1AF";
    auto req_ = led_color_client_->async_send_request(color_request);
    req_.wait();
    response->success = true;
    response->message = "Soft Armed";
    return;
  }
  soft_arm_ = false;
  target_vel_yaw = 0;
  target_vel_heave = 0;
  target_vel_surge = 0;
  auto color_request =
      std::make_shared<auv_interfaces::srv::SetColor::Request>();
  color_request->color = "#FFC0CB";
  auto req_ = led_color_client_->async_send_request(color_request);
  req_.wait();
  response->success = true;
  response->message = "Soft Disarmed";
}

// Handle the angle correction service call
void DumbController::handle_angle_correction(
    const std::shared_ptr<auv_interfaces::srv::AngleCorrection::Request>
        request,
    std::shared_ptr<auv_interfaces::srv::AngleCorrection::Response> response) {
  angle_correction_enabled_ = request->enable;

  if (angle_correction_enabled_) {
    // Store the current yaw angle when enabled
    stored_yaw_ = current_yaw_;
    RCLCPP_INFO(this->get_logger(),
                "Angle correction enabled, stored yaw: %.2f degrees",
                stored_yaw_);
  } else {
    RCLCPP_INFO(this->get_logger(), "Angle correction disabled");
  }

  response->success = true;
  response->message = angle_correction_enabled_
                          ? "Angle correction enabled, reference yaw stored"
                          : "Angle correction disabled";
}
void DumbController::set_mode(const std::string &mode) {
  if (!mode_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;

  auto callback =
      [this,
       mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        auto result = future.get();
        if (result->mode_sent) {
          RCLCPP_INFO(this->get_logger(), "Successfully set mode to: %s",
                      mode.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to set mode to: %s",
                       mode.c_str());
        }
      };

  mode_client_->async_send_request(request, callback);
}
void DumbController::send_calibration_request() {
  // Create a request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send the request
  auto result_future = calibration_client_->async_send_request(
      request, std::bind(&DumbController::calibration_callback, this,
                         std::placeholders::_1));
}

void DumbController::calibration_callback(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
  static int retry_count = 0;
  const int max_retries = 3;
  const int retry_delay_ms = 1000;
  
  try {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Calibration successful: %s",
                  response->message.c_str());
      retry_count = 0; // Reset counter on success
    } else {
      RCLCPP_ERROR(this->get_logger(), "Calibration failed: %s",
                   response->message.c_str());
      
      // Retry logic
      if (retry_count < max_retries) {
        retry_count++;
        RCLCPP_WARN(this->get_logger(), "Retrying calibration (%d/%d)...",
                   retry_count, max_retries);
                   
        // Schedule retry with a delay
        auto timer = this->create_wall_timer(
          std::chrono::milliseconds(retry_delay_ms),
          [this, timer_ref = std::weak_ptr<rclcpp::TimerBase>()] {
            auto timer_ptr = timer_ref.lock();
            if (timer_ptr) {
              timer_ptr->cancel();
            }
            auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
            calibration_client_->async_send_request(
                req, std::bind(&DumbController::calibration_callback, this,
                              std::placeholders::_1));
          });
      } else {
        RCLCPP_ERROR(this->get_logger(), "Calibration failed after %d attempts", 
                     max_retries);
        retry_count = 0; // Reset counter for next time
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    
    // Retry logic for exceptions
    if (retry_count < max_retries) {
      retry_count++;
      RCLCPP_WARN(this->get_logger(), "Retrying calibration after exception (%d/%d)...",
                 retry_count, max_retries);
                 
      // Schedule retry with a delay
      auto timer = this->create_wall_timer(
        std::chrono::milliseconds(retry_delay_ms),
        [this, timer_ref = std::weak_ptr<rclcpp::TimerBase>()] {
          auto timer_ptr = timer_ref.lock();
          if (timer_ptr) {
            timer_ptr->cancel();
          }
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          calibration_client_->async_send_request(
              req, std::bind(&DumbController::calibration_callback, this,
                           std::placeholders::_1));
        });
    } else {
      RCLCPP_ERROR(this->get_logger(), "Calibration failed after %d attempts due to exceptions", 
                   max_retries);
      retry_count = 0; // Reset counter for next time
    }
  }
}