cmake_minimum_required(VERSION 3.8)
project(turtlebot3_lio_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models rviz urdf DESTINATION share/${PROJECT_NAME})
ament_package()
