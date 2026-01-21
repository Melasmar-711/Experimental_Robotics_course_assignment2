#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mutex>

using namespace std::chrono_literals;

class PictureAction : public plansys2::ActionExecutorClient {
public:
  PictureAction() : plansys2::ActionExecutorClient("take_picture", 250ms) {
    RCLCPP_INFO(get_logger(), "Initializing PictureAction...");

    callback_group_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber_;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 
      rclcpp::SensorDataQoS(), 
      std::bind(&PictureAction::image_callback, this, std::placeholders::_1),
      sub_opt);
    
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    parameters_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(get_logger(), "PictureAction Initialized.");
  }

  // PUBLIC METHOD: To be called by the main thread only
  void show_image() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (has_image_to_display_ && !display_image_.empty()) {
      cv::imshow("Robot Camera", display_image_);
      cv::waitKey(1); // Required to handle window events
    }
  }

private:
  enum State { IDLE, SEARCHING, APPROACHING };
  State state_ = IDLE;
  
  void do_work() override {
    auto args = get_arguments();
    if (args.size() < 2) {
      finish(false, 0.0, "Missing marker argument");
      return;
    }

    std::string marker_str = args[1];
    try {
        if (marker_str[0] == 'm') {
            target_marker_id_ = std::stoi(marker_str.substr(1));
        } else {
            target_marker_id_ = std::stoi(marker_str);
        }
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "Invalid marker ID: %s", marker_str.c_str());
        finish(false, 0.0, "Invalid ID");
        return;
    }

    if (state_ == IDLE) {
      state_ = SEARCHING;
      reached_marker_ = false;
      RCLCPP_INFO(get_logger(), "Looking for marker %d...", target_marker_id_);
    }

    if (reached_marker_) {
      stop_robot();
      RCLCPP_INFO(get_logger(), "SUCCESS: Photo taken of marker %d.", target_marker_id_);
      finish(true, 1.0, "Photo taken");
      state_ = IDLE;
      target_marker_id_ = -1;
      return;
    }

    if (state_ == SEARCHING) {
      auto cmd = geometry_msgs::msg::Twist();
      cmd.angular.z = 0.4; 
      cmd_vel_pub_->publish(cmd);
      send_feedback(0.3, "Searching for marker...");
    } else if (state_ == APPROACHING) {
      send_feedback(0.7, "Approaching marker...");
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;
        
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, parameters_);

        // 1. Drawing Visuals
        // Draw the title
        std::string title = "Searching for Id: " + (target_marker_id_ == -1 ? "NONE" : std::to_string(target_marker_id_));
        cv::putText(frame, title, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        // Check if target is in sight
        auto it = std::find(ids.begin(), ids.end(), target_marker_id_);
        if (it != ids.end() && target_marker_id_ != -1) {
          int idx = std::distance(ids.begin(), it);
          
          // Draw green box and ID
          cv::aruco::drawDetectedMarkers(frame, corners, ids, cv::Scalar(0, 255, 0));

          // Calculate and draw red center dot
          float center_x = (corners[idx][0].x + corners[idx][2].x) / 2.0;
          float center_y = (corners[idx][0].y + corners[idx][2].y) / 2.0;
          cv::circle(frame, cv::Point2f(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);

          // Centering and Approach Logic (only if not finished)
          if (!reached_marker_) {
            state_ = APPROACHING;
            double area = cv::contourArea(corners[idx]);
            auto cmd = geometry_msgs::msg::Twist();

            if (area < 15000.0) { 
              cmd.linear.x = 0.15; 
              float img_center = frame.cols / 2.0;
              cmd.angular.z = (img_center - center_x) * 0.002;
              cmd_vel_pub_->publish(cmd);
            } else {
              reached_marker_ = true;
            }
          }
        } else if (state_ == APPROACHING && !reached_marker_) {
          state_ = SEARCHING;
        }

        // 2. Safely hand off frame to main thread
        {
          std::lock_guard<std::mutex> lock(image_mutex_);
          frame.copyTo(display_image_);
          has_image_to_display_ = true;
        }

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void stop_robot() {
    auto cmd = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(cmd);
    cv::destroyWindow("Robot Camera");
  }

  bool reached_marker_ = false;
  int target_marker_id_ = -1;
  
  std::mutex image_mutex_;
  cv::Mat display_image_;
  bool has_image_to_display_ = false;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PictureAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "take_picture"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // Use a manual spin loop to keep the GUI alive on the main thread
  while (rclcpp::ok()) {
    executor.spin_some();
    node->show_image();
    std::this_thread::sleep_for(10ms);
  }

  rclcpp::shutdown();
  return 0;
}