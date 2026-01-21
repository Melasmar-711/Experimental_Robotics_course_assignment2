cmake_minimum_required(VERSION 3.8)
project(plansys_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
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
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(search_action_node src/search_action_node.cpp)
add_executable(picture_action_node src/picture_action_node.cpp)
add_executable(move_action_node src/move_action_node.cpp)
add_executable(action_manager_node src/action_manager_node.cpp)

ament_target_dependencies(search_action_node
  rclcpp
  plansys2_executor
  geometry_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  nav2_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(search_action_node ${OpenCV_LIBRARIES})

ament_target_dependencies(picture_action_node
  rclcpp
  plansys2_executor
  cv_bridge
  sensor_msgs
  geometry_msgs
)
target_link_libraries(picture_action_node ${OpenCV_LIBRARIES})

ament_target_dependencies(move_action_node
  rclcpp
  rclcpp_action
  plansys2_executor
  plansys2_msgs
  geometry_msgs
  nav2_msgs
)

ament_target_dependencies(action_manager_node
  rclcpp
  plansys2_executor
  plansys2_msgs
  plansys2_domain_expert
  plansys2_planner
  plansys2_problem_expert
  plansys2_pddl_parser
  std_msgs
)

install(DIRECTORY
  pddl
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  search_action_node picture_action_node  move_action_node action_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
