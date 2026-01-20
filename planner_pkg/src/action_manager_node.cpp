#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // For receiving marker data from search node

// Struct to track markers discovered during Phase 1
struct MarkerInfo {
  int id;
  std::string waypoint;
};

class ActionManager : public rclcpp::Node
{
public:
  ActionManager() : rclcpp::Node("action_manager"), state_(PHASE_1_SEARCH)
  {
    // Initialize marker subscriber
    // search_action_node should publish strings like "ID:WP" (e.g., "15:wp1")
    marker_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/detected_markers", 10, std::bind(&ActionManager::marker_callback, this, std::placeholders::_1));
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    // Wait for the services to be ready
    rclcpp::Rate loop_rate(1);
    while (!problem_expert_->addInstance(plansys2::Instance("robot1", "robot"))) {
    RCLCPP_INFO(get_logger(), "Waiting for PlanSys2 stack to become ACTIVE...");
    rclcpp::spin_some(this->get_node_base_interface());
    loop_rate.sleep();
    }

    // Step 1: Set Phase 1 Goal
    problem_expert_->setGoal(plansys2::Goal("(and (searched wp1) (searched wp2) (searched wp3) (searched wp4))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {
      auto result = executor_client_->getResult();

      if (result.has_value() && result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan sequence completed successfully.");

        if (state_ == PHASE_1_SEARCH) {
          setup_phase_2();
        } else {
          RCLCPP_INFO(get_logger(), "MISSION COMPLETE.");
          rclcpp::shutdown();
        }
      } else if (result.has_value() && !result.value().success) {
        RCLCPP_ERROR(get_logger(), "Plan failed. Retrying...");
        start_execution();
      }
    }
  }

  void start_execution()
  {
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR(get_logger(), "Could not find plan!");
      return;
    }

    executor_client_->start_plan_execution(plan.value());
  }

private:
  void marker_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Parse marker ID and Waypoint from search_action_node
    // Expected format "id:wp"
    std::string data = msg->data;
    size_t sep = data.find(':');
    if (sep != std::string::npos) {
      int id = std::stoi(data.substr(0, sep));
      std::string wp = data.substr(sep + 1);
      
      // Check if already recorded
      auto it = std::find_if(found_markers_.begin(), found_markers_.end(), 
                [id](const MarkerInfo& m) { return m.id == id; });
      if (it == found_markers_.end()) {
        found_markers_.push_back({id, wp});
        RCLCPP_INFO(get_logger(), "Discovered Marker %d at %s", id, wp.c_str());
      }
    }
  }

  void setup_phase_2()
  {
    RCLCPP_INFO(get_logger(), "Transitioning to Phase 2: Inspection.");
    state_ = PHASE_2_INSPECTION;

    // 1. Sort markers by ID ascending
    std::sort(found_markers_.begin(), found_markers_.end(), 
              [](const MarkerInfo& a, const MarkerInfo& b) { return a.id < b.id; });

    // 2. Add markers to PDDL Problem
    std::string goal_str = "(and ";
    for (const auto& marker : found_markers_) {
      std::string m_name = "m" + std::to_string(marker.id);
      
      // Add instance
      problem_expert_->addInstance(plansys2::Instance(m_name, "marker"));
      
      // Add location predicate (marker_at marker waypoint)
      problem_expert_->addPredicate(plansys2::Predicate("(marker_at " + m_name + " " + marker.waypoint + ")"));
      
      // Add to goal string
      goal_str += "(photo_taken " + m_name + ") ";
    }
    goal_str += ")";

    // 3. Update Goal and re-execute
    problem_expert_->setGoal(plansys2::Goal(goal_str));
    start_execution();
  }

  enum State { PHASE_1_SEARCH, PHASE_2_INSPECTION } state_;
  std::vector<MarkerInfo> found_markers_;
  
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
    node->step();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}