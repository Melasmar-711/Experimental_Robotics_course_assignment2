#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/action/execute_plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

using namespace std::chrono_literals;

enum Phase { EXPLORATION, ORDERED_PROCESSING, FINISHED };

// Helper to print plan
std::ostream& operator<<(std::ostream& os, const plansys2_msgs::msg::Plan & plan) {
    os << "Generated Plan:\n";
    for (const auto & item : plan.items) {
        os << "  Action: " << item.action << " (Duration: " << item.duration << ", Start Time: " << item.time << ")\n";
    }
    return os;
}

class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller"), current_phase_(EXPLORATION) {}

    void init() {
        domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
        planner_client_ = std::make_shared<plansys2::PlannerClient>();
        problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
        
        execute_plan_client_ = rclcpp_action::create_client<plansys2_msgs::action::ExecutePlan>(
            this, "execute_plan");

        RCLCPP_INFO(get_logger(), "Phase 1: Exploration...");
        
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();

        if (problem.empty()) {
            RCLCPP_ERROR(get_logger(), "Problem is empty!");
            return;
        }

        auto plan = planner_client_->getPlan(domain, problem);
        if (!plan.has_value()) {
            RCLCPP_ERROR(get_logger(), "Could not find plan for Phase 1!");
            return;
        }

        std::cout << plan.value() << std::endl;
        execute_plan(plan.value());
    }

private:
    void execute_plan(const plansys2_msgs::msg::Plan & plan) {
        if (!execute_plan_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(get_logger(), "Executor action server not available");
            return;
        }

        auto goal_msg = plansys2_msgs::action::ExecutePlan::Goal();
        goal_msg.plan = plan;

        auto send_goal_options = rclcpp_action::Client<plansys2_msgs::action::ExecutePlan>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            std::bind(&Controller::execution_result_callback, this, std::placeholders::_1);

        execute_plan_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(get_logger(), "Plan sent to executor. Waiting for completion...");
    }

    void execution_result_callback(const rclcpp_action::ClientGoalHandle<plansys2_msgs::action::ExecutePlan>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "Plan Finished Successfully!");
            
            if (current_phase_ == EXPLORATION) {
                transition_to_ordered();
            } else if (current_phase_ == ORDERED_PROCESSING) {
                RCLCPP_INFO(get_logger(), "MISSION COMPLETE: All markers processed.");
                current_phase_ = FINISHED;
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Plan Failed or Cancelled!");
            current_phase_ = FINISHED;
        }
    }

    void transition_to_ordered() {
        RCLCPP_INFO(get_logger(), "-----------------------------------------");
        RCLCPP_INFO(get_logger(), "Transitioning to Phase 2: Ordering Markers");

        // 1. Get Discovered Markers
        auto instances = problem_expert_->getInstances();
        std::vector<int> ids;
        std::stringstream ss;
        
        ss << "Found: ";
        for(const auto & inst : instances) {
            if(inst.type == "marker" && inst.name != "m_start") {
                try {
                    int id = std::stoi(inst.name.substr(6)); // "marker512" -> 512
                    ids.push_back(id);
                    ss << id << " ";
                } catch (...) {}
            }
        }
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

        if (ids.empty()) {
             RCLCPP_ERROR(get_logger(), "No markers found!");
             current_phase_ = FINISHED;
             return;
        }

        // 2. Sort (Lowest -> Highest)
        std::sort(ids.begin(), ids.end());
        
        // Clear StringStream to print ACTUAL sorted order
        ss.str(""); ss.clear(); 
        ss << "Sorted: ";
        for(int id : ids) ss << id << " ";
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

        // 3. Clear Old Goal & Prep Logic
        problem_expert_->clearGoal();
        
        // Add dummy start marker to anchor the chain
        problem_expert_->addInstance(plansys2::Instance("m_start", "marker"));
        problem_expert_->addPredicate(plansys2::Predicate("(processed m_start)"));

        std::string prev = "m_start";
        for(int id : ids) {
            std::string curr = "marker" + std::to_string(id);
            // Define the order: prev must be processed before curr
            problem_expert_->addPredicate(plansys2::Predicate("(next_id " + prev + " " + curr + ")"));
            prev = curr;
        }

        // 4. Set Goal (Wrap in AND to satisfy parser)
        std::string final_marker = "marker" + std::to_string(ids.back());
        
        
        std::string goal_str = "(and (processed " + final_marker + "))"; 
        
        RCLCPP_INFO(get_logger(), "New Goal: %s", goal_str.c_str());
        
        if (problem_expert_->setGoal(plansys2::Goal(goal_str))) {
             RCLCPP_INFO(get_logger(), "Goal set successfully.");
        } else {
             RCLCPP_ERROR(get_logger(), "Failed to set goal: %s", goal_str.c_str());
             current_phase_ = FINISHED;
             return;
        }

        // 5. Generate Phase 2 Plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (plan.has_value()) {
            current_phase_ = ORDERED_PROCESSING;
            std::cout << plan.value() << std::endl;
            execute_plan(plan.value());
        } else {
            RCLCPP_ERROR(get_logger(), "Phase 2 Planning Failed! (Check predicates/types)");
        }
    }

    Phase current_phase_;
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    rclcpp_action::Client<plansys2_msgs::action::ExecutePlan>::SharedPtr execute_plan_client_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  node->init();
  rclcpp::spin(node); 
  rclcpp::shutdown();
  return 0;
}