# revopoint_ros
revopoint ros package.

tested revopoint 2

Windows version is currently available

## Install dependencies
**ROS**
+ Please refer directly to ROS [wiki](https://wiki.ros.org/ROS/Installation)
+ windows ROS install [wiki](https://wiki.ros.org/Installation/Windows)


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
## TODO
- [ ] linux version Complete
- [ ] Add parameter options ex) Rgb Auto Exposure
- [ ] Support for other products ex) revopoint3
