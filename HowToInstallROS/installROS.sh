# Installing MlAgents
#git clone -b release_10_branch https://github.com/Unity-Technologies/ml-agents
#cd mlagents
#pip3 install .




sudo apt update
sudo apt upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh
chmod 755 ./install_ros_melodic.sh 
bash ./install_ros_melodic.sh


# Installing ROS!
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
ros-melodic-rosserial-server ros-melodic-rosserial-client \
ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
ros-melodic-compressed-image-transport ros-melodic-rqt* \
ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

sudo apt-get remove ros-melodic-dynamixel-sdk
sudo apt-get remove ros-melodic-turtlebot3-msgs
sudo apt-get remove ros-melodic-turtlebot3
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
# -b melodic-devel
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b big-mecanum https://github.com/zhl017/turtlebot3_msgs.git
git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git

cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
