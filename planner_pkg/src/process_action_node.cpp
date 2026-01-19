#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <mutex> // Required for thread safety

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp> // Check if your system needs .h or .hpp
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

class ProcessAction : public plansys2::ActionExecutorClient {
public:
  ProcessAction() : plansys2::ActionExecutorClient("process_marker", 100ms) {
    // Constructor empty to avoid segfaults
  }

private:
  void do_work() override {
    // --- Lazy Initialization ---
    if (cmd_vel_pub_ == nullptr) {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    if (image_sub_ == nullptr) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image", 10, 
            std::bind(&ProcessAction::image_callback, this, std::placeholders::_1));
    }
    // ---------------------------

    // 1. Safe Image Copy (Fixes the Crash)
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

    // 2. Get Target ID from arguments
    if (get_arguments().size() < 2) {
        finish(false, 1.0, "Missing arguments");
        return;
    }

    std::string arg_id = get_arguments()[1];
    int target_id = -1;
    try {
        target_id = std::stoi(arg_id.substr(6)); // Strip "marker" prefix
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "Invalid marker argument: %s", arg_id.c_str());
        finish(false, 1.0, "Invalid Argument");
        return;
    }

    // 3. Detect Markers (Using the Safe Copy)
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(frame_copy, dictionary, corners, ids);

    bool found = false;
    float center_x = 0.0;

    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == target_id) {
            found = true;
            // Calculate center X of the marker
            center_x = (corners[i][0].x + corners[i][2].x) / 2.0f;
            break;
        }
    }

    geometry_msgs::msg::Twist vel;
    if (found) {
        // 4. Visual Servoing
        float image_center = frame_copy.cols / 2.0f;
        float error = center_x - image_center;

        // Threshold (e.g., within 10 pixels)
        if (std::abs(error) > 10.0) {
            vel.angular.z = -0.002 * error; // P-Controller
            cmd_vel_pub_->publish(vel);
            send_feedback(0.5, "Centering marker...");
        } else {
            // 5. Processing Action (Stop & Save)
            vel.angular.z = 0.0;
            cmd_vel_pub_->publish(vel);

            RCLCPP_INFO(get_logger(), "Marker %d centered. Processing...", target_id);

            // Draw Green Circle on the COPY
            cv::circle(frame_copy, cv::Point(center_x, frame_copy.rows / 2), 50, cv::Scalar(0, 255, 0), 4);
            
            // Save Image
            std::string filename = "marker_" + std::to_string(target_id) + "_processed.jpg";
            cv::imwrite(filename, frame_copy);
            RCLCPP_INFO(get_logger(), "Saved image: %s", filename.c_str());

            finish(true, 1.0, "Marker Processed");
        }
    } else {
        // If target marker not seen, rotate slowly to find it
        vel.angular.z = 0.3;
        cmd_vel_pub_->publish(vel);
        send_feedback(0.2, "Searching for target marker...");
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try { 
        std::lock_guard<std::mutex> lock(img_mutex_);
        last_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image; 
    } 
    catch (...) {}
  }

  std::mutex img_mutex_; // Mutex for thread safety
  cv::Mat last_frame_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{nullptr};
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