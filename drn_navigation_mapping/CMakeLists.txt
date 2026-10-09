cmake_minimum_required(VERSION 3.8)

project(drn_navigation_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_waypoint_follower REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(navigation2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(rviz2 REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(drn_navigation_msgs REQUIRED) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/libexample.cpp
  #...
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(${PROJECT_NAME}_node src/nodes/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_directories(${PROJECT_NAME}_node PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/lib)

ament_target_dependencies(${PROJECT_NAME}_node 
  OpenCV 
  rclcpp 
  std_msgs 
  nav2_common
  nav2_util
  nav2_msgs
  nav2_waypoint_follower
  nav2_controller
  nav2_bt_navigator
  navigation2
  gazebo_ros
  sensor_msgs geometry_msgs 
  slam_toolbox
  lifecycle_msgs
  rviz2
  nav2_behaviors
  cv_bridge image_transport 
  drn_navigation_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY 
  launch
  config
  maps
  scripts
  DESTINATION share/${PROJECT_NAME})  

ament_package()
