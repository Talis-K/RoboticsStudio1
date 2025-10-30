cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros_ign_interfaces REQUIRED)


ament_python_install_package(${PROJECT_NAME})

# Install executable scripts
install(PROGRAMS
  41068_ignition_bringup/lidar_processing/filtered_lidar.py
  41068_ignition_bringup/lidar_processing/tree_detector.py
  41068_ignition_bringup/gui/gui_panel.py
  41068_ignition_bringup/odom/odometry_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 41068_ignition_bringup/config 41068_ignition_bringup/launch 41068_ignition_bringup/models 41068_ignition_bringup/urdf 41068_ignition_bringup/worlds 41068_ignition_bringup/urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
