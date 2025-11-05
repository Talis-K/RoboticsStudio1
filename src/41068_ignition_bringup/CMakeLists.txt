cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) 
find_package(ros_ign_interfaces REQUIRED)

# Install the Python module subpackage 
ament_python_install_package(lidar_processing)
ament_python_install_package(chainsaw_detector2)


# Install executable scripts 
install(PROGRAMS
  lidar_processing/filtered_lidar.py
  lidar_processing/tree_detector.py
  gui/gui_panel.py
  drone_control/odometry_listener.py
  chainsaw_detector2/chainsaw_detector_node.py 
  chainsaw_detector2/test_audio_publisher.py
  main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
