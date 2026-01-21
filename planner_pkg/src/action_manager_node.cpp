#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <chrono>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

struct MarkerInfo {
  int id;
  std::string waypoint;
};

class ActionManager : public rclcpp::Node
{
public:
  ActionManager() : rclcpp::Node("action_manager"), state_(PHASE_1_SEARCH), current_marker_idx_(0)
  {
    marker_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/detected_markers", 10, std::bind(&ActionManager::marker_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "Action Manager Node Started.");
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    // Wait for the PlanSys2 stack to be ready
    rclcpp::Rate loop_rate(1);
    while (rclcpp::ok() && !problem_expert_->addInstance(plansys2::Instance("robot1", "robot"))) {
      RCLCPP_INFO(get_logger(), "Waiting for PlanSys2 stack to become ACTIVE...");
      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
    }
  }

  bool step()
  {
    if (!executor_client_->execute_and_check_plan()) {
      auto result = executor_client_->getResult();

      if (result.has_value() && result.value().success) {
        if (state_ == PHASE_1_SEARCH) {
          setup_phase_2();
        } else if (state_ == PHASE_2_INSPECTION) {
          return proceed_to_next_marker();
        }
      } else if (result.has_value() && !result.value().success) {
        RCLCPP_ERROR(get_logger(), "Plan failed or timed out. Retrying...");
        start_execution();
      }
    }
    return true;
  }

  void start_execution()
  {
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "Could not find plan! Check if robot position is known.");
      return;
    }

    executor_client_->start_plan_execution(plan.value());
  }

private:
  void marker_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string data = msg->data;
    size_t sep = data.find(':');
    if (sep != std::string::npos) {
      int id = std::stoi(data.substr(0, sep));
      std::string wp = data.substr(sep + 1);
      
      auto it = std::find_if(found_markers_.begin(), found_markers_.end(), 
                [id](const MarkerInfo& m) { return m.id == id; });
      if (it == found_markers_.end()) {
        found_markers_.push_back({id, wp});
      }
    }
  }

  void setup_phase_2()
  {
    RCLCPP_INFO(get_logger(), "Phase 1 Complete. Sorting markers and starting Phase 2...");
    state_ = PHASE_2_INSPECTION;

    // 1. Sort markers by ID ascending
    std::sort(found_markers_.begin(), found_markers_.end(), 
              [](const MarkerInfo& a, const MarkerInfo& b) { return a.id < b.id; });

    // 2. Add all discovered markers as instances
    for (const auto& marker : found_markers_) {
      std::string m_name = "m" + std::to_string(marker.id);
      problem_expert_->addInstance(plansys2::Instance(m_name, "marker"));
      problem_expert_->addPredicate(plansys2::Predicate("(marker_at " + m_name + " " + marker.waypoint + ")"));
      RCLCPP_INFO(get_logger(), "Registered Marker %s at %s", m_name.c_str(), marker.waypoint.c_str());
    }

    // Give PlanSys2 a moment to process instances before requesting a plan
    std::this_thread::sleep_for(500ms);

    current_marker_idx_ = 0;
    set_goal_for_current_marker();
  }

  bool proceed_to_next_marker()
  {
    current_marker_idx_++;
    if (current_marker_idx_ < found_markers_.size()) {
      set_goal_for_current_marker();
      return true;
    } else {
      RCLCPP_INFO(get_logger(), "MISSION COMPLETE: All markers processed in ascending order.");
      return false; // Exit loop
    }
  }

  void set_goal_for_current_marker()
  {
    std::string m_name = "m" + std::to_string(found_markers_[current_marker_idx_].id);
    RCLCPP_INFO(get_logger(), "Targeting Marker: %s (ID: %d)", m_name.c_str(), found_markers_[current_marker_idx_].id);
    
    // FIX: Always wrap the goal in (and ...) to avoid "Malformed Expression" errors
    std::string goal_str = "(and (photo_taken " + m_name + "))";
    
    if (!problem_expert_->setGoal(plansys2::Goal(goal_str))) {
        RCLCPP_ERROR(get_logger(), "Failed to set goal: %s", goal_str.c_str());
    }
    
    start_execution();
  }

  enum State { PHASE_1_SEARCH, PHASE_2_INSPECTION } state_;
  std::vector<MarkerInfo> found_markers_;
  size_t current_marker_idx_;
  
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr marker_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionManager>();
  node->init();
  node->start_execution();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    if (!node->step()) {
      break; 
    }
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Shutting down...");
  rclcpp::shutdown();
  return 0;
}