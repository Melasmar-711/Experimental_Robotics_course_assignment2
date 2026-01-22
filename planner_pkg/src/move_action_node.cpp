#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <memory>
#include <chrono>
#include <string>
#include <map>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms), progress_(0.0), goal_sent_(false)
  {
    odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&MoveAction::odom_callback, this, std::placeholders::_1)
    );

    nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      nav2_node_, "navigate_to_pose"
    );

    waypoints_["wp0"] = {1.0, 1.0};   
    waypoints_["wp1"] = {-6.0, -6.0};
    waypoints_["wp2"] = {-6.0, 6.0};
    waypoints_["wp3"] = {6.0, -6.0};
    waypoints_["wp4"] = {6.0, 6.0};
  }

private:
  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() < 3) {
      RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
      finish(false, 0.0, "Insufficient arguments");
      return;
    }

    // args[0] is robot, args[1] is wp_from, args[2] is wp_to
    std::string wp_to_navigate = args[2];

    if (waypoints_.find(wp_to_navigate) == waypoints_.end()) {
      RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", wp_to_navigate.c_str());
      finish(false, 0.0, "Unknown waypoint");
      return;
    }

    double goal_x = waypoints_[wp_to_navigate].first;
    double goal_y = waypoints_[wp_to_navigate].second;

    if (!goal_sent_) {
      if (!nav2_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
        return;
      }

      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "map";
      goal_pose.header.stamp = now();
      goal_pose.pose.position.x = goal_x;
      goal_pose.pose.position.y = goal_y;
      goal_pose.pose.orientation.w = 1.0;

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = goal_pose;

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      RCLCPP_INFO(get_logger(), "Navigating to waypoint: %s", wp_to_navigate.c_str());
      send_goal_options.result_callback =
        [this, wp_to_navigate](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
        {
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(get_logger(), "Navigation failed to waypoint: %s", wp_to_navigate.c_str());
            finish(false, 0.0, "Nav2 failed");
          } else {
            RCLCPP_INFO(get_logger(), "Reached waypoint: %s", wp_to_navigate.c_str());
            finish(true, 1.0, "Move completed");
          }
          goal_sent_ = false; 
        };

      nav2_client_->async_send_goal(goal_msg, send_goal_options);
      goal_sent_ = true;

      start_x_ = current_x_;
      start_y_ = current_y_;
    }

    // Progress feedback calculation based on distance
    double total_dist = std::hypot(goal_x - start_x_, goal_y - start_y_);
    double rem_dist   = std::hypot(goal_x - current_x_, goal_y - current_y_);
    progress_ = total_dist > 0.01 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;

    send_feedback(progress_, "Moving to " + wp_to_navigate);

    rclcpp::spin_some(nav2_node_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }

  std::map<std::string, std::pair<double, double>> waypoints_;
  float progress_;
  bool goal_sent_;
  double start_x_ = 1.0, start_y_ = 1.0;
  double current_x_ = 1.0, current_y_ = 1.0;

  rclcpp::Node::SharedPtr nav2_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}