cmake_minimum_required(VERSION 3.8)
project(pibot_globalplan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pibot_msg REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(aco_test src/aco_test.cpp)
ament_target_dependencies(aco_test
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pibot_msg
)

add_executable(biaco src/biaco.cpp)
ament_target_dependencies(biaco
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pibot_msg
)
install(TARGETS
  aco_test
  biaco
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/planner_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
