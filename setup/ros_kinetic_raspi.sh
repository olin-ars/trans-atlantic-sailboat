sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update

sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake dirmngr

sudo rosdep init
rosdep update

rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch
num_cpu_cores=`grep -c "^processor" /proc/cpuinfo`
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j3

source /opt/ros/kinetic/setup.bash
echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc

mkdir -p ~/catkin_workspace/src
cd catkin_workspace/src
catkin_init_workspace
git clone https://github.com/ros/common_msgs.git

cd ~/catkin_workspace/

catkin_make

source ~/catkin_workspace/devel/setup.bash

echo 'source ~/catkin_workspace/devel/setup.bash' >> ~/.bashrc
