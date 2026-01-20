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
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&PictureAction::image_callback, this, std::placeholders::_1));
    
    // FIX: Initialize pointer correctly
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    RCLCPP_INFO(get_logger(), "PictureAction Initialized.");
  }

private:
  void do_work() override {
    auto args = get_arguments();
    if (args.size() < 2) return;

    std::string marker_str = args[1];
    try {
        if (marker_str[0] == 'm') {
            target_marker_id_ = std::stoi(marker_str.substr(1));
        } else {
            target_marker_id_ = std::stoi(marker_str);
        }
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "Invalid marker ID format: %s", marker_str.c_str());
        return;
    }
    
    if (reached_marker_) {
      RCLCPP_INFO(get_logger(), "Taking photo of marker %d", target_marker_id_);
      finish(true, 1.0, "Photo taken");
      reached_marker_ = false;
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        
        // FIX: Pass the pointer
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);

        auto it = std::find(ids.begin(), ids.end(), target_marker_id_);
        if (it != ids.end()) {
          int idx = std::distance(ids.begin(), it);
          double area = cv::contourArea(corners[idx]);
          
          auto cmd = geometry_msgs::msg::Twist();
          
          // Debugging log (optional)
          // RCLCPP_INFO(get_logger(), "Area: %f", area);

          if (area < 12000.0) { 
            cmd.linear.x = 0.2; 
            float center_x = (corners[idx][0].x + corners[idx][2].x) / 2.0;
            float img_center = cv_ptr->image.cols / 2.0;
            cmd.angular.z = (img_center - center_x) * 0.002;
            cmd_vel_pub_->publish(cmd);
          } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            reached_marker_ = true;
          }
        }
    } catch (cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV Error: %s", e.what());
    }
  }

  bool reached_marker_ = false;
  int target_marker_id_ = -1;
  // FIX: Must be cv::Ptr
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PictureAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "take_picture"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}