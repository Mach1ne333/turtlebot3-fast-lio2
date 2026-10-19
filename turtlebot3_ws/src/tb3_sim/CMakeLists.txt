cmake_minimum_required(VERSION 3.8)
project(tb3_sim)


find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models rviz urdf config DESTINATION share/${PROJECT_NAME})
# Install the user's original resources without modifying or duplicating models.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../../gazebo_building_editor_models"
  DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ekf_2d test/test_ekf_2d.py)
  ament_add_pytest_test(test_package_identity test/test_package_identity.py)
  ament_add_pytest_test(test_shiyanlou_ramp test/test_shiyanlou_ramp.py)
endif()
ament_package()
