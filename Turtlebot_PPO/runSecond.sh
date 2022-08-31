#!/bin/bash
source /home/jetbot/catkin_ws/devel/setup.bash
echo jetbot | sudo -S chmod a+rw /dev/ttyACM0
roslaunch turtlebot3_bringup turtlebot3_core.launch

