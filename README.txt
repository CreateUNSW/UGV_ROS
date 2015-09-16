Welcome to the UGV_ROS repository.

In order to get this code working, you must install ROS

Install ROS on any linux OS, preferably Ubuntu Trusty (14.04).

Create a catkin workspace using the instructions at this site: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Clone this repository into (your_catkin_ws)/src

All done! Use rosrun or roslaunch to start running programs

A quick note about dependencies:
 - controller_get needs to have output piped to it from 'xboxdrv -d --quiet'
   (http://pingus.seul.org/~grumbel/xboxdrv/)

source dev/setup.bash in 3 terminals
in first terminal run: roscore
in second terminal run: sudo xboxdrv --detach-kernel-driver --quiet | rosrun ugv_nav controller_get
in third terminal run: rosrun ugv_nav motordata_arduino_send /dev/ttyACM0

if arduino has used a different port, in third terminal run: ls /dev | grep ttyACM
