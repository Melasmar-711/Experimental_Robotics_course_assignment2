#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>

using namespace std::chrono_literals;

class SearchAction : public plansys2::ActionExecutorClient {
public:
  SearchAction() : plansys2::ActionExecutorClient("search_waypoint", 250ms) {
    RCLCPP_INFO(get_logger(), "Initializing SearchAction...");

    callback_group_subscriber_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_subscriber_;

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    marker_pub_ = this->create_publisher<std_msgs::msg::String>("/detected_markers", 10);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 
      rclcpp::SensorDataQoS(), 
      std::bind(&SearchAction::image_callback, this, std::placeholders::_1),
      sub_opt); 

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, 
      std::bind(&SearchAction::odom_callback, this, std::placeholders::_1),
      sub_opt);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    parameters_ = cv::aruco::DetectorParameters::create();

    RCLCPP_INFO(get_logger(), "SearchAction Initialized.");
  }

  // PUBLIC METHOD: This must be called from the main thread loop
  void show_image() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (has_image_to_display_ && !display_image_.empty()) {
      cv::imshow("Robot Camera", display_image_);
      cv::waitKey(1); // Required to process GUI events
    }
  }

private:
  void do_work() override {
    auto args = get_arguments();
    if (args.size() < 2) {
        finish(false, 0.0, "Missing arguments");
        return;
    }
    current_waypoint_ = args[1]; 

    if (status_ == IDLE) {
      initial_yaw_ = current_yaw_;
      rotated_360_ = false;
      marker_found_ = false; 
      status_ = ROTATING;
      RCLCPP_INFO(get_logger(), "Starting search at %s", current_waypoint_.c_str());
    }

    if (marker_found_) {
      stop_robot();
      RCLCPP_INFO(get_logger(), "Marker detected! Finishing action for %s", current_waypoint_.c_str());
      finish(true, 1.0, "Marker found");
      status_ = IDLE;
      return; 
    }

    auto cmd = geometry_msgs::msg::Twist();
    if (status_ == ROTATING) {
      cmd.angular.z = 0.5; 
      cmd_vel_pub_->publish(cmd);

      double diff = std::abs(current_yaw_ - initial_yaw_);
      if (diff > M_PI) diff = 2 * M_PI - diff; 
      
      if (std::abs(current_yaw_ - initial_yaw_) > 6.0) rotated_360_ = true;
      
      if (rotated_360_ && diff < 0.2) {
        stop_robot();
        RCLCPP_INFO(get_logger(), "360 rotation complete at %s (No marker found)", current_waypoint_.c_str());
        finish(true, 1.0, "Waypoint searched (complete)");
        status_ = IDLE;
      }
    }
    send_feedback(0.5, "Scanning " + current_waypoint_);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (status_ != ROTATING || marker_found_) return;
    
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      
      cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);
      
      // Visual overlays
      cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
      std::string id_text = ids.empty() ? "NONE" : std::to_string(ids[0]);
      cv::putText(cv_ptr->image, "Detected ID: " + id_text,
                  cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

      // Thread-safe handoff to the main thread for display
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        cv_ptr->image.copyTo(display_image_);
        has_image_to_display_ = true;
      }

      if (!ids.empty()) {
        auto message = std_msgs::msg::String();
        message.data = std::to_string(ids[0]) + ":" + current_waypoint_;
        marker_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "SUCCESS: Found Marker %d at %s", ids[0], current_waypoint_.c_str());
        marker_found_ = true;
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p;
    m.getRPY(r, p, current_yaw_);
  }

  void stop_robot() {
    auto stop_cmd = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(stop_cmd);
  }

  enum { IDLE, ROTATING } status_ = IDLE;
  std::string current_waypoint_;
  double initial_yaw_ = 0.0, current_yaw_ = 0.0;
  bool rotated_360_ = false;
  bool marker_found_ = false; 

  std::mutex image_mutex_;
  cv::Mat display_image_;
  bool has_image_to_display_ = false;
  
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "search_waypoint"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // CUSTOM SPIN LOOP: Handles ROS callbacks AND OpenCV GUI rendering
  while (rclcpp::ok()) {
    executor.spin_some(); // Process ROS messages
    node->show_image();   // Process OpenCV GUI (Main thread only)
    std::this_thread::sleep_for(10ms); 
  }

  rclcpp::shutdown();
  return 0;
}