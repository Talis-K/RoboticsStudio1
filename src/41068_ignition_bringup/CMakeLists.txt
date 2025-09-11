cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

ament_python_install_package(subpackage_1)

# Install executable scripts (for ros2 run)
install(PROGRAMS
  subpackage_1/filtered_lidar.py
  subpackage_1/tree_detector.py
  subpackage_1/gui_panel.py  
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (configs, launch files, etc.)

install(
  DIRECTORY config launch models urdf worlds urdf_drone
  DESTINATION share/${PROJECT_NAME}
)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
