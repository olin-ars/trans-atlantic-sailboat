#!/bin/bash

# TODO Make this support arm64 as well
required_arch="armv6l"
ros_version="kinetic"
repo_root=$(pwd)
catkin_root=${repo_root}/../

# Check that we're on an ARM processor
arch=$(uname -m)
if [[ "$arch" != "$required_arch" ]]
then
  echo "Sorry, ARM is the only architecture supported by this setup script."
  exit
fi

##### INSTALL ROS #####

# Add the apt repo and ROS public key
echo "Adding ROS package repo to apt..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install dirmngr
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
echo Installing package updates...
sudo apt-get -y autoremove
sudo apt-get -y upgrade

echo Package updates completed

##### INSTALL PYTHON PACKAGES #####

# Make sure pip is installed
echo Installing Python pip...
sudo apt-get -y install python-pip python-dev
sudo -H pip install --upgrade python pip

# Install ROS Python packages (for all Python 2 interpreters)
sudo -H pip install testresources
sudo apt-get -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential cmake

# Initialize rosdep
sudo rosdep init
rosdep update

echo "Preparing ROS installation..."
cd ${catkin_root}
rosinstall_generator ros_comm --rosdistro ${ros_version} --deps --wet-only --exclude roslisp --tar > ${ros_version}-ros_comm-wet.rosinstall
echo
echo "Installing ROS..."
wstool init src ${ros_version}-ros_comm-wet.rosinstall
rm ${ros_version}-ros_comm-wet.rosinstall

# Build dependencies not available precompiled
echo "Building dependencies..."
mkdir external_src
cd external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
rm assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install

echo
echo "Dependency compilation complete"
echo

echo "Downloading extra needed ROS message types not included by default..."
cd ${catkin_root}
git clone https://github.com/ros/common_msgs.git
cd common_msgs
# Remove unnecessary modules to avoid building them (which can take a long time)
rm -rf actionlib_msgs diagnostic_msgs nav_msgs shape_msgs stereo_msgs trajectory_msgs visualization_msgs

echo
echo "Download complete"
echo

echo "Building ROS..."
cd ${catkin_root}
rosdep install -y --from-paths src --ignore-src --rosdistro ${ros_version} -r --os=debian:$(lsb_release -sc)
# Took 6 hours on Raspberry Pi Zero (before skipping unnecessary dependencies)
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/${ros_version}
echo
echo "ROS compilation complete"
echo

# Load the ROS environment variables upon opening a new shell
echo ""  >> ~/.bashrc
echo "# ROS" >> ~/.bashrc
echo "source /opt/ros/$ros_version/setup.bash" >> ~/.bashrc

# Add the repo to the Python interpreter path so it can find the Python modules we make
echo "export PYTHONPATH=\$PYTHONPATH:$(repo_root)" >> ~/.bashrc

# Load all environment variables, aliases, etc
source $HOME/.bashrc

# Install Python dependencies
echo
echo "Installing Python requirements..."
cd ${repo_root}
sudo -H pip install -r requirements.txt

echo
echo "Python configuration complete"
echo

##### BUILD THE CATKIN WORKSPACE #####

echo "Building the catkin workspace..."

cd ../../
catkin_make

# Load the catkin workspace stuff when loading the virtual environment
echo "source $(catkin_root)/devel/setup.bash" >> ~/.bashrc
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
cd ${repo_root}/../../
rm -rf DynamixelSDK
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
