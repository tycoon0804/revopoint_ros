# revopoint_ros
revopoint ros package. Windows version is currently available

## Install dependencies
**ROS**

## Getting start
+ Create a ros workspace
```
mkdir ros_ws\src
```
+ Clone code from github.
```
cd ros_ws\src
git clone https://github.com/tycoon0804/revopoint_ros.git
```
+ Build
```
cd ..
catkin_make
```
+ Start the camera
```
devel\setup.bat
roslaunch revopoint revopoint.launch
```
