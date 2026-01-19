cmake_minimum_required(VERSION 3.8)
project(plansys_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include OpenCV headers globally or per target
include_directories(${OpenCV_INCLUDE_DIRS})

# Define executables
add_executable(get_plan_and_execute src/getplan_and_execute.cpp)
add_executable(move_action_node src/move_action_node.cpp)
add_executable(search_waypoint_node src/search_waypoint.cpp)
add_executable(process_action_node src/process_action_node.cpp)

# Controller Node dependencies (The Coordinator)
ament_target_dependencies(get_plan_and_execute
  rclcpp
  plansys2_pddl_parser
  plansys2_msgs
  plansys2_domain_expert
  plansys2_planner
  plansys2_problem_expert
  plansys2_executor
  rclcpp_action
)

# Move Action dependencies (Navigation)
ament_target_dependencies(move_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
)

# Search Action dependencies (Discovery + OpenCV)
ament_target_dependencies(search_waypoint_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_problem_expert
  plansys2_msgs
  sensor_msgs
  cv_bridge
)
# Explicitly link OpenCV libraries
target_link_libraries(search_waypoint_node ${OpenCV_LIBRARIES})

# Process Action dependencies (Visual Servoing + OpenCV)
ament_target_dependencies(process_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)
# Explicitly link OpenCV libraries
target_link_libraries(process_action_node ${OpenCV_LIBRARIES})

# Installation
install(DIRECTORY domain launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  get_plan_and_execute
  move_action_node
  search_waypoint_node
  process_action_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()