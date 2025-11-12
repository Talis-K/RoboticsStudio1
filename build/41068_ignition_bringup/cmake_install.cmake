# Install script for directory: /home/talis/RoboticsStudio1/src/41068_ignition_bringup

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/talis/RoboticsStudio1/install/41068_ignition_bringup")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/environment" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/environment" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/lidar_processing-1.0.3-py3.10.egg-info" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_python/lidar_processing/lidar_processing.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/lidar_processing" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/lidar_processing/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/talis/RoboticsStudio1/install/41068_ignition_bringup/local/lib/python3.10/dist-packages/lidar_processing"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/drone_control-1.0.3-py3.10.egg-info" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_python/drone_control/drone_control.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/drone_control" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/drone_control/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/talis/RoboticsStudio1/install/41068_ignition_bringup/local/lib/python3.10/dist-packages/drone_control"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/path_planning-1.0.3-py3.10.egg-info" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_python/path_planning/path_planning.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/path_planning" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/path_planning/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/talis/RoboticsStudio1/install/41068_ignition_bringup/local/lib/python3.10/dist-packages/path_planning"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/chainsaw_detector2-1.0.3-py3.10.egg-info" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_python/chainsaw_detector2/chainsaw_detector2.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/chainsaw_detector2" TYPE DIRECTORY FILES "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/chainsaw_detector2/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3" "-m" "compileall"
        "/home/talis/RoboticsStudio1/install/41068_ignition_bringup/local/lib/python3.10/dist-packages/chainsaw_detector2"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/41068_ignition_bringup" TYPE PROGRAM FILES
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/lidar_processing/filtered_lidar.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/lidar_processing/tree_detector.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/lidar_processing/all_lidar_processing.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/gui/gui_panel.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/drone_control/odometry_listener.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/drone_control/dronecontrolling.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/chainsaw_detector2/chainsaw_detector_node.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/chainsaw_detector2/test_audio_publisher.py"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/main.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE DIRECTORY FILES
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/config"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/launch"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/models"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/urdf"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/worlds"
    "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/urdf_drone"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/41068_ignition_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/41068_ignition_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/environment" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/environment" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_index/share/ament_index/resource_index/packages/41068_ignition_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup/cmake" TYPE FILE FILES
    "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_core/41068_ignition_bringupConfig.cmake"
    "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/ament_cmake_core/41068_ignition_bringupConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/41068_ignition_bringup" TYPE FILE FILES "/home/talis/RoboticsStudio1/src/41068_ignition_bringup/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/talis/RoboticsStudio1/build/41068_ignition_bringup/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
