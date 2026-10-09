cmake_minimum_required(VERSION 3.8)
project(drn_navigation_decision_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -std=c++20)
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
# find_package(frequency_check_msg REQUIRED)
find_package(nav2_msgs REQUIRED) # Adicionado para o novo arquivo
find_package(rclcpp_action REQUIRED) # Adicionado para suportar ações
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(siemens_et200sp_modbus_driver_msgs REQUIRED)

# Create list of all source files
set(SOURCES
    src/states/state.cpp
    src/states/off_state.cpp
    src/states/idle_state.cpp
    src/states/navigation_state.cpp
    src/states/docking_state.cpp
    src/decision_maker_node.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${drn_navigation_msgs_INCLUDE_DIRS}
  ${drn_integration_msgs_INCLUDE_DIRS}
  ${drn_perception_msgs_INCLUDE_DIRS}
  # ${frequency_check_msg_INCLUDE_DIRS}
)

# Create the library
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

# Set library properties
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add library dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  tf2
  tf2_ros  
  tf2_geometry_msgs 
  nav2_costmap_2d 
  behaviortree_cpp_v3
  drn_navigation_msgs
  drn_integration_msgs
  drn_perception_msgs
  siemens_et200sp_modbus_driver_msgs
  # frequency_check_msg
)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/decision_maker_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Export targets
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  std_msgs
  nav2_msgs
)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install Python scripts
install(
  PROGRAMS scripts/test_decision_maker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
