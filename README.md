rbe501Team6_Haptics
====================
This repository only contains our additions to existing work. To install the required ROS packages follow the process outlined below:


# DVRK Install 
* Download and install cisst-saw from here: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros
* Start from section 1.3 on that page. The code below should be the necessary steps to follow from the web-page cited above
* install "catkin-tools" if you don't have them yet
```sh
sudo apt-get install python-catkin-tools
```

* Create a workspace for, and then install, Cisst-Saw (which uses catkin build)
```sh
mkdir -p ~/cisst_ws/src
cd ~/cisst_ws/src
catkin init

# install ciss-saw
cd ~/cisst_ws/src
git clone https://github.com/jhu-cisst/cisst-saw --recursive

# configure and build
cd ~/cisst_ws
# make sure you have the proper ROS environment variables
source /opt/ros/indigo/setup.bash
# create a profile named debug with _release extension
catkin config --profile release -x _release
# switch to newly created release profile
catkin profile set release
# set default CMake build type
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
# build
catkin build
# set environment variables - MAKE SURE YOU SOURCE THE RIGHT FILE (debug vs. release)
source devel_release/setup.bash
```
* Now you can use the standard catkin_ws (or any other name you want) for the WPI-DVRK components (which use catkin_make)
```sh
cd ~/catkin_ws/src
git clone https://github.com/WPI-AIM/dvrk-ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
* Now you should be able to teleoperate the PSM/MTM system
```sh
# Launch Teleoperation Simulator
roslaunch dvrk_teleop test_teleop.launch
```
# DVRK 	Personalization
* Source the packages in the following order in your ~/.bashrc file
```sh
source ~/cisst_ws/devel_release/setup.bash
source ~/catkin_ws/devel/setup.bash
```

# WPI-DVRK-ROS (Haptics Packages)
* to get the haptics package installed use this code:
```sh
cd ~/catkin_ws/src
git clone https://github.com/WPI-AIM/wpi-dvrk-ros
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
* If you get a build error about _moveit_commander_ then you need to add that package

```sh
sudo apt-get install ros-indigo-moveit
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

# Haptics-Feedback Force Data
* To obtain haptics force feedback data and see the visualization in your RVIZ simulator, follow these steps.
* Modify your _teleop_haptics_half_ launch file as follows. You need to disable these lines (Lines 5-6) of code as they cause conflicts with the _joint_state_pubisher_ slider GUI.

```
<!-- Launch motion planning/kinematics nodes -->
	<!--node name="teleop" pkg="dvrk_teleop" type="dvrk_teleop" /-->
```
* Run _teleop_haptics_half_ launch.
* Using the _Scene Objects_ tab in the RVIZ simulator, upload your .STL file for any object that will be in the workspace.
* Move and scale as desired, and then _Export As Text_ which will create a .scene file. It would help to store this file in a separate location. Clear the scene.
* Import your object/setup using _Import From Text_.
* Under the _Context Tab_, _Publish Current Scene_.
* In a different terminal, run the desired ROS Topic, i.e.

```
rostopic echo /dvrk_psm/haptics_feedback_force
```
* To obtain a normalized set of force vectors and the visualization in the workspace. Note: If a spherical object is utilized as an obstacle, it can be seen that there are sudden jumps in force values, as the normal vector keeps changing radically because of how contact points are defined.
