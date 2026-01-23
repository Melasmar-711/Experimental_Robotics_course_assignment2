#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <thread>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp> // Required for imshow

using namespace std::chrono_literals;

class ProcessAction : public plansys2::ActionExecutorClient {
public:
  ProcessAction() : plansys2::ActionExecutorClient("process_marker", 100ms) {
    // Constructor
  }

private:
  void do_work() override {
    // --- Lazy Initialization ---
    if (cmd_vel_pub_ == nullptr) {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot_vel", 10);
    }
    if (image_sub_ == nullptr) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, 
            std::bind(&ProcessAction::image_callback, this, std::placeholders::_1));
    }

    // 1. Safe Image Copy
    cv::Mat frame_copy;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (last_frame_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for image data...");
            send_feedback(0.0, "Waiting for image...");
            return;
        }
        last_frame_.copyTo(frame_copy);
    }

    // 2. Get Target ID
    if (get_arguments().size() < 2) {
        finish_action(false, "Missing arguments");
        return;
    }

    std::string arg_id = get_arguments()[1];
    int target_id = -1;
    try {
        target_id = std::stoi(arg_id.substr(6)); // Strip "marker" prefix
    } catch (...) {
        finish_action(false, "Invalid Argument");
        return;
    }

    // 3. Detect Markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(frame_copy, dictionary, corners, ids);

    // Draw bounding boxes for ALL markers
    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame_copy, corners, ids);
    }

    bool found = false;
    float center_x = 0.0;
    float center_y = 0.0;

    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == target_id) {
            found = true;
            // Calculate exact center of the marker
            center_x = (corners[i][0].x + corners[i][2].x) / 2.0f;
            center_y = (corners[i][0].y + corners[i][2].y) / 2.0f;
            break;
        }
    }

    geometry_msgs::msg::Twist vel;
    bool action_finished = false;

    // --- LOGIC BLOCK ---
    if (found) {
        float image_center_x = frame_copy.cols / 2.0f;
        float error = center_x - image_center_x;

        // Check if we are already in the "Locked & Waiting" phase
        if (processing_started_) {
            // STOP ROBOT
            vel.angular.z = 0.0;
            cmd_vel_pub_->publish(vel);

            // Draw GREEN Circle on the marker center
            cv::circle(frame_copy, cv::Point(center_x, center_y), 50, cv::Scalar(0, 255, 0), 4);
            cv::putText(frame_copy, "LOCKED", cv::Point(center_x - 30, center_y - 60), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Check Timer
            auto elapsed = (this->now() - processing_start_time_).seconds();
            int remaining = 5 - (int)elapsed;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Marker locked. Finishing in %d seconds...", remaining);

            if (elapsed > 5.0) {
                action_finished = true;
            }

        } else {
            // Not locked yet, check error
            if (std::abs(error) > 10.0) {
                // SERVOING
                vel.angular.z = -0.002 * error; 
                cmd_vel_pub_->publish(vel);
                send_feedback(0.5, "Centering marker...");
                
                // Draw YELLOW Circle (Adjusting) on marker center
                cv::circle(frame_copy, cv::Point(center_x, center_y), 50, cv::Scalar(0, 255, 255), 2);
            
            } else {
                // CENTERED! Start the 5-second timer
                processing_started_ = true;
                processing_start_time_ = this->now();
                
                vel.angular.z = 0.0;
                cmd_vel_pub_->publish(vel);
                
                RCLCPP_INFO(get_logger(), "Marker centered! Locking for 5 seconds...");
            }
        }
    } else {
        // Lost marker?
        if (processing_started_) {
            // If we lost it while waiting, just count down anyway
            vel.angular.z = 0.0;
            cmd_vel_pub_->publish(vel);
            
            auto elapsed = (this->now() - processing_start_time_).seconds();
            if (elapsed > 5.0) action_finished = true;
            
        } else {
            // Search Mode
            vel.angular.z = 0.3;
            cmd_vel_pub_->publish(vel);
            send_feedback(0.2, "Searching for target marker...");
        }
    }

    // --- SHOW WINDOW ---
    try {
        cv::imshow("Process Action", frame_copy);
        cv::waitKey(1); 
    } catch (...) {}

    if (action_finished) {
        finish_action(true, "Marker Processed");
    }
  }

  void finish_action(bool success, std::string msg) {
      // Stop Robot
      geometry_msgs::msg::Twist vel;
      vel.angular.z = 0.0;
      if(cmd_vel_pub_) cmd_vel_pub_->publish(vel);

      // Reset State
      processing_started_ = false;

      // Close Window
      try {
          cv::destroyAllWindows();
          cv::waitKey(1); 
      } catch (...) {}
      
      finish(success, 1.0, msg);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try { 
        std::lock_guard<std::mutex> lock(img_mutex_);
        last_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image; 
    } catch (...) {}
  }

  std::mutex img_mutex_;
  cv::Mat last_frame_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{nullptr};
  
  // Timer variables
  bool processing_started_ = false;
  rclcpp::Time processing_start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ProcessAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "process_marker"));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}