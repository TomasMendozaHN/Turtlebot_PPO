# Steps taken:

# -- When turning on, to enable the GUI: sudo systemctl set-default graphical.target
# -- Then, write: sudo reboot 

# When you reach the Desktop and have connected to internet:

###############1: Disable Docker *** if using JetBot image only ###############  

cd ~/jetbot/docker
./disable.sh


###############2: Create Swap Memory (for more RAM) ###############

cd 
sudo fallocate -l 4G /var/swapfile
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile
sudo bash -c 'echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab'


###############3: Install Python + Numpy + Cython ###############

sudo apt-get update
sudo apt install -y python3-pip python3-pil
sudo -H pip3 install Cython
sudo -H pip3 install --upgrade numpy


###############4: Install jtop (to check memory consumption and temp of Nano) ###############

sudo -H pip3 install jetson-stats 


###############5: Install the pre-built TensorFlow pip wheel ###############
sudo apt-get update
sudo apt-get install -y libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
sudo apt-get install -y python3-pip
sudo pip3 install -U pip testresources setuptools numpy==1.16.1 future==0.17.1 mock==3.0.5 h5py==2.9.0 keras_preprocessing==1.0.5 keras_applications==1.0.8 gast==0.2.2 futures protobuf pybind11
# TF-1.15
sudo pip3 install --pre --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v44 'tensorflow<2'


###############6: Install Pytorch 1.6 ###############

wget https://nvidia.box.com/shared/static/9eptse6jyly1ggt9axbja2yrmj6pbarc.whl -O torch-1.6.0-cp36-cp36m-linux_aarch64.whl
sudo apt-get install python3-pip libopenblas-base libopenmpi-dev 
pip3 install Cython
pip3 install numpy torch-1.6.0-cp36-cp36m-linux_aarch64.whl


###############7: Install Torchvision 0.7.0 ###############

sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch release/0.7 https://github.com/pytorch/vision torchvision 
cd torchvision
export BUILD_VERSION=0.7.0
python3 setup.py install --user
cd ../
pip install 'pillow<7'



###############8: Install Torch2TrT ###############

cd $HOME
git clone https://github.com/NVIDIA-AI-IOT/torch2trt
cd torch2trt
sudo python3 setup.py install


###############9: Install Traitlets ###############

sudo python3 -m pip install git+https://github.com/ipython/traitlets@dead2b8cdde5913572254cf6dc70b5a6065b86f8


###############10: Install Jupyter Lab & Configure It! ###############

sudo apt install -y curl
wget https://nodejs.org/dist/v12.13.0/node-v12.13.0-linux-arm64.tar.xz
tar -xJf node-v12.13.0-linux-arm64.tar.xz
cd node-v12.13.0-linux-arm64
sudo cp -R * /usr/local/
node -v   # Here you should see a print that says "node js version 12.X"
sudo -H pip3 install jupyter jupyterlab
sudo -H jupyter labextension install @jupyter-widgets/jupyterlab-manager
jupyter lab --generate-config
python3 -c "from notebook.auth.security import set_password; set_password('$password', '$HOME/.jupyter/jupyter_notebook_config.json')"

# fix for permission error
sudo chown -R jetbot:jetbot ~/.local/share/

# Install jupyter_clickable_image_widget
cd
sudo apt-get install libssl1.0-dev
git clone https://github.com/jaybdub/jupyter_clickable_image_widget
cd jupyter_clickable_image_widget
git checkout tags/v0.1
sudo -H pip3 install -e .
sudo jupyter labextension install js
sudo jupyter lab build

# Install bokeh
sudo apt-get install jupyter_bokeh

###############11. install jetbot python module ###############
cd
sudo apt install -y python3-smbus
git clone https://github.com/NVIDIA-AI-IOT/jetbot
cd ~/jetbot
sudo apt-get install -y cmake
sudo python3 setup.py install 

###############12. Install jetbot services (for remote connection) ###############
cd jetbot/utils
python3 create_stats_service.py
sudo mv jetbot_stats.service /etc/systemd/system/jetbot_stats.service
sudo systemctl enable jetbot_stats
sudo systemctl start jetbot_stats
python3 create_jupyter_service.py
sudo mv jetbot_jupyter.service /etc/systemd/system/jetbot_jupyter.service
sudo systemctl enable jetbot_jupyter
sudo systemctl start jetbot_jupyter


###############13. Install python gst dependencies ###############
sudo apt-get install -y \
    libwayland-egl1 \
    gstreamer1.0-plugins-bad \
    libgstreamer-plugins-bad1.0-0 \
    gstreamer1.0-plugins-good \
    python3-gst-1.0
    
###############14. Install zmq dependency (should actually already be resolved by jupyter) ###############
sudo -H pip3 install pyzmq
    

###############15. Optimize the system configuration to create more headroom ###############
sudo nvpmodel -m 0
sudo systemctl set-default multi-user
sudo systemctl disable nvzramconfig.service

###############16. Copy JetBot notebooks to home directory ###############
cp -r ~/jetbot/notebooks ~/Notebooks

###############17. Install TRT_POSE ##########################

sudo pip3 install tqdm cython pycocotools
sudo apt-get install python3-matplotlib

git clone https://github.com/NVIDIA-AI-IOT/trt_pose
cd trt_pose
sudo python3 setup.py install

##############18. Install RPLidar ###########################
sudo pip3 install rplidar



################ Installing pyrealsense2 #######################

sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-setuptools \
    python3-pip \
	python3-dev

# Install the core packages required to build librealsense libs
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
# Install Distribution-specific packages for Ubuntu 18
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Install LibRealSense from source
# We need to build from source because
# the PyPi pip packages are not compatible with Arm processors.
# See link [here](https://github.com/IntelRealSense/librealsense/issues/6964).

# First clone the repository
git clone https://github.com/IntelRealSense/librealsense.git
cd ./librealsense

# Make sure that your RealSense cameras are disconnected at this point
# Run the Intel Realsense permissions script
./scripts/setup_udev_rules.sh

# Now the build
mkdir build && cd build
## Install CMake with Python bindings (that's what the -DBUILD flag is for)
## see link: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#building-from-source
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
## Recompile and install librealsense binaries
## This is gonna take a while! The -j4 flag means to use 4 cores in parallel
## but you can remove it and simply run `sudo make` instead, which will take longer
sudo make uninstall && sudo make clean && sudo make -j4 && sudo make install

## Export pyrealsense2 to your PYTHONPATH so `import pyrealsense2` works
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2


############### IMPORTANT ##################

sudo systemctl set-default graphical.target


