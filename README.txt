Last edited: 15/10/2015

Welcome to the UGV_ROS repository.

In order to get this code working, you must install ROS

Install ROS indigo on any linux OS, preferably Ubuntu Trusty (14.04).

THIS REPOSITORY is a catkin workspace. To build, use catkin_make.

All done! Use rosrun or roslaunch to start running programs

Useful commands:
roscore
sudo xboxdrv --detach-kernel-driver --quiet | rosrun ugv_nav controller_get
rosrun ugv_nav motordata_arduino_send /dev/ttyACM0

if arduino has used a different port, in third terminal run: ls /dev | grep ttyACM

A launch file is in the process of being built. Stay tuned for further run instructions.

