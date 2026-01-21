#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

class PictureAction : public plansys2::ActionExecutorClient {
public:
  PictureAction() : plansys2::ActionExecutorClient("take_picture", 250ms) {
    RCLCPP_INFO(get_logger(), "Initializing PictureAction...");

    // 1. Callback Group to prevent PlanSys2 heartbeat starvation
    callback_group_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber_;

    // 2. Publishers & Subscriptions
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", // Ensure this matches your camera topic
      rclcpp::SensorDataQoS(), 
      std::bind(&PictureAction::image_callback, this, std::placeholders::_1),
      sub_opt);
    
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    parameters_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(get_logger(), "PictureAction Initialized and Ready.");
  }

private:
  enum State { IDLE, SEARCHING, APPROACHING };
  State state_ = IDLE;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
  
  void do_work() override {
    auto args = get_arguments();
    if (args.size() < 2) {
      finish(false, 0.0, "Missing marker argument");
      return;
    }

    // Parse Target Marker ID (from PDDL object name like "m1")
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

    // INITIALIZATION: Start searching if we just began
    if (state_ == IDLE) {
      state_ = SEARCHING;
      reached_marker_ = false;
      RCLCPP_INFO(get_logger(), "Looking for marker %d...", target_marker_id_);
    }

    // SUCCESS CHECK: If the callback signaled we are close enough
    if (reached_marker_) {
      stop_robot();
      RCLCPP_INFO(get_logger(), "SUCCESS: Photo taken of marker %d.", target_marker_id_);
      finish(true, 1.0, "Photo taken");
      state_ = IDLE;
      target_marker_id_ = -1;
      return;
    }

    // ROTATION LOGIC: If we are in SEARCHING state, keep rotating
    if (state_ == SEARCHING) {
      auto cmd = geometry_msgs::msg::Twist();
      cmd.angular.z = 0.4; // Rotate to find the marker
      cmd_vel_pub_->publish(cmd);
      send_feedback(0.3, "Searching for marker...");
    } else if (state_ == APPROACHING) {
      send_feedback(0.7, "Approaching marker...");
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (target_marker_id_ == -1 || reached_marker_) return;

    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);

        auto it = std::find(ids.begin(), ids.end(), target_marker_id_);
        
        if (it != ids.end()) {
          // --- MARKER FOUND ---
          state_ = APPROACHING;
          int idx = std::distance(ids.begin(), it);
          double area = cv::contourArea(corners[idx]);
          
          auto cmd = geometry_msgs::msg::Twist();

          // If marker is still small, move closer
          if (area < 15000.0) { 
            cmd.linear.x = 0.15; 
            
            // Centering logic (Visual Servoing)
            float center_x = (corners[idx][0].x + corners[idx][2].x) / 2.0;
            float img_center = cv_ptr->image.cols / 2.0;
            cmd.angular.z = (img_center - center_x) * 0.002;
            
            cmd_vel_pub_->publish(cmd);
          } else {
            // Close enough!
            reached_marker_ = true;
          }
        } else {
          // --- MARKER LOST (OR NOT YET FOUND) ---
          if (state_ == APPROACHING) {
            RCLCPP_WARN(get_logger(), "Lost marker %d! Resuming search...", target_marker_id_);
            state_ = SEARCHING;
          }
        }
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV: %s", e.what());
    }
  }

  void stop_robot() {
    auto cmd = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(cmd);
  }

  bool reached_marker_ = false;
  int target_marker_id_ = -1;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
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
  executor.spin();

  rclcpp::shutdown();
  return 0;
}