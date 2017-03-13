rbe501Team6_Haptics
====================
This repository only contains our additions to existing work. To instal the required ros packages follow the process outlined below:


# DVRK Install 
* Download and instal cisst-saw from here: https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake#13-building-using-catkin-build-tools-for-ros
* Start from section 1.3 on that page. The code below should be the necessary steps to follow from the web-page cited above
* instal "catkin-tools" if you don't have them yet
```sh
sudo apt-get install python-catkin-tools
```

* Create a workspace for, and then install, Ciss-Saw (which uses catkin build)
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
* Now you can use the standard catkin_ws for the WPI-DVRK components (which use catkin_make)
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
# DVRK Personalization
* Source the packages in the following order in your ~/.bashrc file
```sh
source ~/cisst_ws/devel_release/setup.bash
source ~/catkin_ws/devel/setup.bash
```
