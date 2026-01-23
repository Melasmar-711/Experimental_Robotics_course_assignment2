#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

using namespace std::chrono_literals;

class SearchAction : public plansys2::ActionExecutorClient {
public:
  SearchAction() : plansys2::ActionExecutorClient("search", 200ms) {
    // Empty constructor
  }

private:
  void do_work() override {
    // 1. Lazy Initialization
    if (problem_expert_ == nullptr) {
      problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    }
    if (image_sub_ == nullptr) {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10, 
        std::bind(&SearchAction::image_callback, this, std::placeholders::_1));
    }
    if (vel_pub_ == nullptr) {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot_vel", 10);
    }

    // 2. Initialize Timer
    if (!search_started_) {
        start_time_ = this->now();
        search_started_ = true;
        RCLCPP_INFO(get_logger(), "Starting rotating search...");
    }

    // 3. Safe Image Copy
    cv::Mat frame_copy;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (last_frame_.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for camera images...");
            return;
        }
        last_frame_.copyTo(frame_copy);
    }

    // 4. Check Arguments
    if (get_arguments().size() < 2) {
        finish_action(false, "Missing arguments");
        return;
    }
    std::string current_wp = get_arguments()[1];

    // 5. Detect Markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(frame_copy, dictionary, corners, ids);

    // --- VISUALIZATION (With Try-Catch) ---
    try {
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame_copy, corners, ids);
        }
        cv::imshow("Robot Camera", frame_copy);
        cv::waitKey(1);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV Error: %s", e.what());
    }
    // --------------------------------------

    // 6. Logic
    if (!ids.empty()) {
        // --- FOUND ---
        int id = ids[0];
        RCLCPP_INFO(get_logger(), "### FOUND MARKER %d at %s ###", id, current_wp.c_str());

        // Update Knowledge
        problem_expert_->addInstance(plansys2::Instance("marker" + std::to_string(id), "marker"));
        problem_expert_->addPredicate(plansys2::Predicate("(marker_at marker" + std::to_string(id) + " " + current_wp + ")"));
        
        finish_action(true, "Marker Found");

    } else {
        // --- NOT FOUND ---
        auto elapsed = (this->now() - start_time_).seconds();

        if (elapsed > 20) {
            RCLCPP_INFO(get_logger(), "Search timed out at %s.", current_wp.c_str());
            finish_action(true, "Search complete (Nothing found)");
        } else {
            rotate_robot();
            send_feedback(elapsed / 20, "Scanning & Rotating...");
        }
    }
  }

  // Helper to cleanly finish the action
  void finish_action(bool success, std::string msg) {
      stop_robot();       // 1. Stop Moving
      
      // 2. CRITICAL: Destroy window to prevent crash during idle time
      try {
        cv::destroyAllWindows();
        cv::waitKey(1); // Process the close event
      } catch (...) {}

      search_started_ = false; // Reset timer flag
      finish(success, 1.0, msg);
  }

  void rotate_robot() {
      geometry_msgs::msg::Twist vel;
      vel.angular.z = 0.4;
      vel_pub_->publish(vel);
  }

  void stop_robot() {
      if (vel_pub_ != nullptr) {
          geometry_msgs::msg::Twist vel;
          vel.linear.x = 0.0;
          vel.angular.z = 0.0;
          vel_pub_->publish(vel);
      }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try { 
        std::lock_guard<std::mutex> lock(img_mutex_);
        last_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image; 
    } catch (...) {}
  }

  std::mutex img_mutex_;
  cv::Mat last_frame_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_{nullptr};
  
  rclcpp::Time start_time_;
  bool search_started_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "search"));
  
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}