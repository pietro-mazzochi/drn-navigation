cmake_minimum_required(VERSION 3.8)
project(docking_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(drn_navigation_msgs REQUIRED)
find_package(drn_integration_msgs REQUIRED)
find_package(drn_perception_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)


add_executable(docking_services_node src/docking_services.cpp)
ament_target_dependencies(docking_services_node 
  rclcpp
  rclcpp_action 
  nav2_msgs
  nav_msgs
  tf2
  tf2_ros  
  tf2_geometry_msgs 
  nav2_costmap_2d 
  drn_navigation_msgs
  drn_perception_msgs
)

install(TARGETS 
  docking_services_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/
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
