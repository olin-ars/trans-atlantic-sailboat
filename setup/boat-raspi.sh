#!/bin/bash

# TODO Make this support arm64 as well
required_arch="armv6l"
ros_version="kinetic"
repo_root=$(pwd)

# Check that we're on an ARM processor
arch=$(uname -m)
if [[ "$arch" != "$required_arch" ]]
then
  echo "Sorry, ARM is the only architecture supported by this setup script."
  exit
fi

##### INSTALL ROS #####

# Add the apt repo and ROS public key
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install dirmngr
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
echo Installing package updates...
sudo apt-get -y autoremove
sudo apt-get -y upgrade

echo Package updates completed

# Install ROS
sudo apt-get -y install ros-${ros_version}-ros-base

# Initialize rosdep
sudo rosdep init
rosdep update

##### INSTALL PYTHON PACKAGES #####

# Make sure pip is installed
echo Installing Python pip...
sudo apt-get -y install python-pip python-dev build-essential
sudo -H pip install --upgrade python pip

# Install ROS Python packages (for all Python 2 interpreters)
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Load the ROS environment variables upon opening a new shell
echo ""  >> ~/.bashrc
echo "# ROS" >> ~/.bashrc
echo "source /opt/ros/$ros_version/setup.bash" >> ~/.bashrc

# Add the repo to the Python interpreter path so it can find the Python modules we make
echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)\n\n" >> ~/.bashrc

# Load all environment variables, aliases, etc
source $HOME/.bashrc

# Install Python dependencies
echo
echo "Installing Python requirements..."
pip install -r requirements.txt

echo
echo "Python configuration complete"
echo

##### BUILD THE CATKIN WORKSPACE #####

echo "Building the catkin workspace..."

cd ../../
catkin_make

# Load the catkin workspace stuff when loading the virtual environment
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
echo ""  >> ~/.bashrc

echo
echo ROS installation complete
echo

#### BUILD DYNAMIXEL SDK ####

cd ${repo_root}/../../

echo "Cloning the Dynamixel SDK source..."
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux_sbc/

echo "Building the Dynamixel drivers..."
make
echo "Build complete. Installing the Dynamixel drivers..."
sudo make install
echo
echo "Dynamixel driver installation complete"

echo
echo "Installing extras..."
echo
sudo apt-get -y install locate screen vim w3m
echo
echo "Updating file index..."
sudo updatedb
echo
echo "Setup complete"
