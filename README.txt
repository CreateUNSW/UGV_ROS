Last edited: 22/02/2016

Welcome to the UGV_ROS repository.

In order to get this code working, you must install ROS

Install ROS indigo on any linux OS, preferably Ubuntu Trusty (14.04).

THIS REPOSITORY is a catkin workspace. To build, use catkin_make.

Launch commands commands:
roscore
sudo xboxdrv --detach-kernel-driver --quiet | rosrun ugv_nav controller_get
roslaunch ugv_nav oweek_ugv.launch

Note: Arduino needs to be on ttyACM0, otherwise please modify motordata_arduino_send.cpp.
if arduino has used a different port, in third terminal run: ls /dev | grep ttyACM

