cmake_minimum_required(VERSION 3.8)
project(custom_bt)

# set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

###
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(behaviortree_cpp REQUIRED)
find_package(drn_navigation_msgs REQUIRED)
###

include_directories(include)

set(dependencies
rclcpp
rclcpp_action
std_msgs
nav2_util
nav2_msgs
nav2_core
nav2_behavior_tree
nav_msgs
geometry_msgs
behaviortree_cpp_v3
###
tf2
tf2_geometry_msgs
tf2_ros
drn_navigation_msgs
# behaviortree_cpp 
###
)

# add_library(${lib_name} SHARED 
#               src/path_longer.cpp)

add_library(path_longer_decorator_bt_node SHARED src/path_longer.cpp)
list(APPEND plugin_libs path_longer_decorator_bt_node)

add_library(remove_in_collision_goals_action_bt_node SHARED src/remove_in_collision_goals_action.cpp)
list(APPEND plugin_libs remove_in_collision_goals_action_bt_node)

add_library(time_selector_bt_node SHARED src/time_selector.cpp)
list(APPEND plugin_libs time_selector_bt_node)

add_library(wait_print_bt_node SHARED src/wait_print.cpp)
list(APPEND plugin_libs wait_print_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS ${lib_name} 
#   DESTINATION lib)
# install(TARGETS ${lib_name} DESTINATION libcustom_bt)

# if(EXISTS "${CMAKE_INSTALL_PREFIX}/libcustom_bt/libcustom_bt_core.so")
#   file(RENAME "${CMAKE_INSTALL_PREFIX}/libcustom_bt/libcustom_bt_core.so"
#                "${CMAKE_INSTALL_PREFIX}/libcustom_bt/PathLonger.so")
# endif()

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

# pluginlib_export_plugin_description_file(behaviortree_cpp_v3 path_longer.xml)
# ament_target_dependencies(${lib_name} ${dep_pkgs} nav2_util geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
