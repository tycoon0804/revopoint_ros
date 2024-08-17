# revopoint_ros
revopoint ros package. Only Windows version is available now

tested revopoint mini 2.

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
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```
+ Start the camera
Connect the USB cable to the camera

Wait until the back side LED is green
```
devel\setup.bat
roslaunch revopoint revopoint.launch
```
## TODO
- [ ] linux version Complete
- [ ] Add parameter options ex) Rgb Auto Exposure
- [ ] Support for other products ex) revopoint pop 3
- [ ] add one shot mode
