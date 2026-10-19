cmake_minimum_required(VERSION 3.16)
project(fast_planner VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system chrono)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

add_library(fast_planner_core SHARED
  planner/localmap_esdf/src/sdf_map.cpp
  planner/trajopt_bspline/src/PlannerInterface.cpp
  planner/trajopt_bspline/src/bspline_replanner.cpp
  planner/trajopt_bspline/src/grad_band_optimizer.cpp
  planner/trajopt_bspline/src/non_uniform_bspline.cpp
  planner/trajopt_bspline/src/bezier_base.cpp
)

target_include_directories(fast_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/localmap_esdf/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/trajopt_bspline/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(fast_planner_core
  PUBLIC
    Eigen3::Eigen
    ${NLOPT_LIBRARIES}
  PRIVATE
    Boost::thread
    Boost::system
    Boost::chrono
)

target_compile_options(fast_planner_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(fast_planner_plan src/trajectory_publisher.cpp)
target_include_directories(fast_planner_plan PRIVATE include)
ament_target_dependencies(fast_planner_plan
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
)
target_link_libraries(fast_planner_plan fast_planner_core)

install(
  TARGETS fast_planner_core
  EXPORT export_fast_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS fast_planner_plan
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY planner/localmap_esdf/include/
  DESTINATION include
)

install(
  DIRECTORY planner/trajopt_bspline/include/
  DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_fast_planner_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 NLopt)

if(BUILD_TESTING)
  option(FAST_PLANNER_LINT_UPSTREAM "Run style checks on the entire legacy source tree" OFF)
  if(FAST_PLANNER_LINT_UPSTREAM)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  else()
    # Integration scope: check new tests and maintained package metadata, not
    # generated ROS 1 headers or unrelated legacy code. Functional tests stay on.
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_uncrustify REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_copyright(test/)
    ament_cpplint(test/)
    ament_uncrustify(test/)
    ament_lint_cmake(CMakeLists.txt)
    ament_xmllint(package.xml)
    # Keep static analysis separate from the approved legacy-style exclusion.
    ament_cppcheck()
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometric_planner test/test_geometric_planner.cpp)
  target_link_libraries(test_geometric_planner fast_planner_core)
endif()

ament_package()
