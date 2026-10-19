cmake_minimum_required(VERSION 3.8)
project(fast_planner_nav2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(fast_planner REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(fast_planner_controller SHARED src/control.cpp src/controller.cpp)
target_include_directories(fast_planner_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fast_planner_controller
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d nav2_util
  nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(fast_planner_controller fast_planner::fast_planner_core)
target_compile_options(fast_planner_controller PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(nav2_core controller_plugin.xml)
install(TARGETS fast_planner_controller EXPORT export_fast_planner_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_fast_planner_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(fast_planner rclcpp rclcpp_lifecycle pluginlib nav2_core
  nav2_costmap_2d nav2_util nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_link_libraries(test_control fast_planner_controller)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller fast_planner_controller)
endif()
ament_package()
