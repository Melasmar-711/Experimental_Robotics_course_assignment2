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

using namespace std::chrono_literals;

class SearchAction : public plansys2::ActionExecutorClient {
public:
  SearchAction() : plansys2::ActionExecutorClient("search_waypoint", 250ms) {
    RCLCPP_INFO(get_logger(), "Initializing SearchAction...");

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    marker_pub_ = this->create_publisher<std_msgs::msg::String>("/detected_markers", 10);
    
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&SearchAction::image_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&SearchAction::odom_callback, this, std::placeholders::_1));
    
    // FIX: Correctly initialize Pointers for OpenCV compatibility
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    parameters_ = cv::aruco::DetectorParameters::create();
    
    RCLCPP_INFO(get_logger(), "SearchAction Initialized.");
  }

private:
  void do_work() override {
    auto args = get_arguments();
    if (args.size() < 2) return;
    current_waypoint_ = args[1]; 

    if (status_ == IDLE) {
      initial_yaw_ = current_yaw_;
      rotated_360_ = false;
      status_ = ROTATING;
      RCLCPP_INFO(get_logger(), "Starting search at %s", current_waypoint_.c_str());
    }

    auto cmd = geometry_msgs::msg::Twist();
    if (status_ == ROTATING) {
      cmd.angular.z = 0.5; 
      cmd_vel_pub_->publish(cmd);

      double diff = std::abs(current_yaw_ - initial_yaw_);
      if (diff > M_PI) diff = 2 * M_PI - diff; // Normalize angle
      
      if (std::abs(current_yaw_ - initial_yaw_) > 6.0) rotated_360_ = true;
      
      if (rotated_360_ && diff < 0.1) {
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        finish(true, 1.0, "Waypoint searched");
        status_ = IDLE;
      }
    }
    send_feedback(0.5, "Scanning " + current_waypoint_);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (status_ != ROTATING) return;
    
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      
      // FIX: Pass the pointer directly
      cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);

      for (int id : ids) {
        auto message = std_msgs::msg::String();
        message.data = std::to_string(id) + ":" + current_waypoint_;
        marker_pub_->publish(message);
        RCLCPP_INFO(get_logger(), "Found Marker: %d", id);
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (cv::Exception& e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double r, p;
    m.getRPY(r, p, current_yaw_);
  }

  enum { IDLE, ROTATING } status_ = IDLE;
  std::string current_waypoint_;
  double initial_yaw_ = 0.0, current_yaw_ = 0.0;
  bool rotated_360_ = false;
  
  // FIX: Must be cv::Ptr to satisfy compiler signature
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  
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
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}