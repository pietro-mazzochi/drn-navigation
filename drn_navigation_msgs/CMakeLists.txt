cmake_minimum_required(VERSION 3.8)
project(drn_navigation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

#find dependencies for GetCosts.srv
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs)
find_package(action_msgs REQUIRED)

nav2_package()

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/GetCosts.srv"
        "action/DockingParam.action"
        "msg/Node.msg"
        "msg/Edge.msg"
        "msg/Graph.msg"
        "msg/RobotStatus.msg"
        "srv/GraphPath.srv"
        DEPENDENCIES builtin_interfaces std_msgs geometry_msgs unique_identifier_msgs action_msgs nav_msgs geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)

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
