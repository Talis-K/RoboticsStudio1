# RoboticStudio1
Repository for RS1 Spring 2025

how to run simulation

1. source ~/RoboticStudio1/install/setup.bash

If you get error with pathways such as:
"CMake Error: The current CMakeCache.txt directory /home/nathan/RoboticStudio1/build/41068_ignition_bringup/CMakeCache.txt is different than the directory /home/student/RoboticStudio1/build/41068_ignition_bringup where CMakeCache.txt was created. This may result in binaries being created in the wrong place. If you are not sure, reedit the CMakeCache.txt"

Use this fix:
git rm -rf build/ install/ log/
colcon build

2. ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=simple_trees


Anytime you edit somthing in the source folder for the simulation (maybe other stuff not sure yet) you must save the file first, then rebuild, otherwise it will not display your updates.

Use:
git rm -rf build/ install/ log/
colcon build

Then you can launch the simulation. 
