#!/bin/bash
source /home/jetbot/catkin_ws/devel/setup.bash
source /home/jetbot/Desktop/1/devel/setup.bash
cd Desktop/Turtlebot_PPO/Turtlebot3
jupyter notebook runPPO.ipynb --ip=0.0.0.0
