#include <memory>
#include <vector>
#include <string>
#include <map>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 500ms)
  {
    // Initialize waypoints coordinates
    // Adjust these coordinates to match your simulation/map
    wp_map_["wp1"] = {-6.0, -6.0};
    wp_map_["wp2"] = {-6.0, 6.0};
    wp_map_["wp3"] = {6.0, -6.0};
    wp_map_["wp4"] = {6.0, 6.0};
    wp_map_["wp_start"] = {0.0, 0.0};
  }

private:
  void do_work() override
  {
    // 1. Handle Arguments
    if (get_arguments().size() < 3) {
      RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
      finish(false, 1.0, "Insufficient arguments");
      return;
    }

    // 2. Lazy Initialization of Nav2 Client
    // We create the client using the node's own interface, no need for a second node.
    if (nav_client_ == nullptr) {
      nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "navigate_to_pose"
      );
    }

    // 3. Wait for Nav2 Server (Non-blocking)
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for Nav2 action server...");
      return;
    }

    // 4. Send Goal (Once)
    if (!goal_sent_) {
      std::string wp_to_navigate = get_arguments()[2];
      
      if (wp_map_.find(wp_to_navigate) == wp_map_.end()) {
        RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", wp_to_navigate.c_str());
        finish(false, 1.0, "Unknown waypoint");
        return;
      }

      double goal_x = wp_map_[wp_to_navigate].first;
      double goal_y = wp_map_[wp_to_navigate].second;

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.pose.position.x = goal_x;
      goal_msg.pose.pose.position.y = goal_y;
      goal_msg.pose.pose.orientation.w = 1.0;

      RCLCPP_INFO(get_logger(), "Sending goal to Nav2: %s (%.1f, %.1f)", wp_to_navigate.c_str(), goal_x, goal_y);

      // Setup Callback
      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.result_callback = 
          std::bind(&MoveAction::result_callback, this, std::placeholders::_1);

      nav_client_->async_send_goal(goal_msg, send_goal_options);
      goal_sent_ = true;
      navigation_finished_ = false;
      navigation_success_ = false;
    }

    // 5. Monitor Status
    if (navigation_finished_) {
      // Reset state for next action
      goal_sent_ = false;
      navigation_finished_ = false;

      if (navigation_success_) {
        finish(true, 1.0, "Move completed");
      } else {
        finish(false, 1.0, "Move failed");
      }
    } else {
      // While moving, just send feedback.
      // We trust Nav2 to tell us when it's done via the result_callback.
      send_feedback(0.5, "Moving...");
    }
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    navigation_finished_ = true;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      navigation_success_ = true;
      RCLCPP_INFO(get_logger(), "Nav2 Success!");
    } else {
      navigation_success_ = false;
      RCLCPP_ERROR(get_logger(), "Nav2 Failed or Cancelled");
    }
  }

  std::map<std::string, std::pair<double, double>> wp_map_;
  bool goal_sent_ = false;
  bool navigation_finished_ = false;
  bool navigation_success_ = false;
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_{nullptr};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  // Ensure this matches the PDDL action name EXACTLY
  node->set_parameter(rclcpp::Parameter("action_name", "move"));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}