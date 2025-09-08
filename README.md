# Important Stuff

This has information on:
* How to correctly clone the repository into you local directory
* How to work on the repository
* How to run the simulation
* How to make sure your changes to the sim are actually being used in sim

## Cloning Repository

To make things more streamlined we need to save the repository in the same location on everyones device.

This will be in the Home folder.

* Change directory to Home:
```bash
cd ~
```

* Clone Repository:
```bash
git clone git@github.com:Talis-K/RoboticsStudio1.git
```

## Working on Repository

## How To Run Simulation

Since we all saved our reposiotries in the same locatio it should be the same code to run the simulation

* Set the source
```bash
source ~/RoboticStudio1/install/setup.bash
```

* Run the simulation
```bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=simple_trees
```


## How to Make Sure Your Changes are in the Simulation

if you edit the world files or model files you need to rebuild the build install and log folders.

* Remove the folders
```bash
git rm -rf build/ install/ log/
```

* Rebuild the folders
```bash
colcon build
```

* Re-run the launch command for the simlation
```bash
ros2 launch 41068_ignition_bringup 41068_ignition_drone.launch.py slam:=true nav2:=true rviz:=true world:=simple_trees
```
 * if you have issues try re-setting the source
 ```bash
source ~/RoboticStudio1/install/setup.bash
```

## Known Errors

This is a list of known erros and their fixes.

### Flashing Simulation

If you continuously get an error like:

```bash
Detected jump back in time. Clearing TF buffer
```

and you probably see things flashing in rviz, then this is probably due to the simulation clock time being reset constantly. This is likely caused by multiple gazebo instances running, perhaps a crashed gazebo in the background that didn't close properly. 

To fix this, I suggest restarting the computer. 

### Source Directory Not Found

If you get error with pathways such as:

```bash
CMake Error: The current CMakeCache.txt directory /home/nathan/RoboticStudio1/build/41068_ignition_bringup/CMakeCache.txt is different than the directory /home/student/RoboticStudio1/build/41068_ignition_bringup where CMakeCache.txt was created. This may result in binaries being created in the wrong place. If you are not sure, reedit the CMakeCache.txt
```

You have your repositorty in the wrong directory.
The correct directory placement is at the top of this page.
I reccomend re-cloning the repository, however you can temperarily get around it by rerbuilding the log build and insatll folders. This methof will give the same issue to everyone else so pls re-clone.

* Remove the folders
```bash
git rm -rf build/ install/ log/
```

* Rebuild the folders
```bash
colcon build
```
