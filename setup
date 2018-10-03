#!/bin/bash

required_ubuntu_version="16.04"
ros_version="kinetic"
repo_root=$(pwd)

# Check that we're on Ubuntu 16.04
ubuntu_version=$(lsb_release -r | tail -c 6)
if [[ "$ubuntu_version" != "$required_ubuntu_version" ]]
then
  echo "Sorry, Ubuntu 16.04 (xenial) is the only version of Ubuntu supported by this setup script."
  exit
fi

##### INSTALL ROS #####

# Add the apt repo and ROS public key
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt -y install dirmngr
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update

# Install ROS
sudo apt-get -y install ros-kinetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

##### INSTALL PYTHON PACKAGES #####

# Make sure pip is installed
sudo apt-get -y install python-pip python-dev build-essential
sudo -H pip install --upgrade python pip virtualenv

# Install ROS Python packages (for all Python 2 interpreters)
sudo apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Make a virtual Python environment
mkdir $repo_root/../.virtualenvs/
virtualenv -p python2.7 --system-site-packages ../../.virtualenvs/oars

# Load the ROS environment variables when using the virtual environment
echo "source /opt/ros/$ros_version/setup.bash" >> ../../.virtualenvs/oars/bin/activate

# Add the repo to the Python interpreter path so it can find the Python modules we make
echo "export PYTHONPATH=\$PYTHONPATH:$repo_root" >> "$repo_root/../.virtualenvs/oars/bin/activate"

# Add a shortcut to activate the virtual environment
venv_activate=$repo_root/../../.virtualenvs/oars/bin/activate
echo "alias useoars=\"source $venv_activate\"" >> $HOME/.bashrc

# Load all environment variables, aliases, etc
source $HOME/.bashrc
echo Loading virtual Python environment...
source $venv_activate

# Install Python dependencies
echo Installing Python requirements...
pip install -r requirements.txt

# Set up pre-commit flake8 hook
flake8 --install-hook git
git config --bool flake8.strict true

echo Python configuration complete

##### BUILD THE CATKIN WORKSPACE #####

echo Building the catkin workspace...

cd ../../
catkin_make

# Load the catkin workspace stuff when loading the virtual environment
echo "source $(pwd)/devel/setup.bash" >> .virtualenvs/oars/bin/activate

# Load all environment variables, aliases, etc
source $venv_activate

echo ROS installation complete

##### INSTALL PYCHARM #####

# Check if the user would like to install PyCharm or configure an existing
# installation for ROS
echo
echo
echo "__________        _________ .__"
echo "\______   \___.__.\_   ___ \|  |__ _____ _______  _____"
echo "|     ___<   |  |/    \  \/|  |  \\__  \\_  __ \/     \\"
echo "|    |    \___  |\     \___|   Y  \/ __ \|  | \/  Y Y  \\"
echo "|____|    / ____| \______  /___|  (____  /__|  |__|_|  /"
echo "           \/             \/     \/     \/            \/"
echo
echo
echo "PyCharm is a wonderful integrated development environment \(IDE\) with a built-in code editor and debugger, the latter being a feature a simple debugger like Atom or Sublime does not have. It is highly recommended you install PyCharm to aid with debugging."
echo
answer_valid=false
install_pycharm=false
configure_pycharm_launcher=false
while [ $answer_valid != true ]; do
  echo What would you like to do?
  echo   a\) Install PyCharm and configure with ROS \(recommended\)
  echo   b\) Configure an existing PyCharm installation for use with ROS
  echo   c\) Do nothing related to PyCharm
  echo -n "Your answer: "
  read answer
  if [ $answer == "A" ] || [ $answer == "a" ] || [ $answer == "" ]
  then
    answer_valid=true
    install_pycharm=true
    configure_pycharm_launcher=true
  elif [ $answer == "B" ] || [ $answer == "b" ]
  then
    answer_valid=true
    configure_pycharm_launcher=true
  elif [ $answer == "C" ] || [ $answer == "c" ]
  then
    answer_valid=true
  else
    echo Sorry, please enter one of the above options.
    echo
  fi
done

if [ $install_pycharm == true ]
then
  echo
  echo -n "Are you a student? [Y/n] "
  read answer
  if [ $answer == "Y" ] || [ $answer == "y" ] || [ $answer == "" ]
  then
    echo Please create an account with JetBrains to get a free copy of PyCharm Professional.
    use_pro=true
    xdg-open https://www.jetbrains.com/student/
    pycharm_version=pycharm-professional
    echo Installing PyCharm Professional...
  else
    pycharm_version=pycharm
    echo Installing PyCharm Community Edition...
  fi
  sudo add-apt-repository -y ppa:ubuntu-desktop/ubuntu-make
  sudo apt-get update
  sudo apt-get -y install ubuntu-make
  umake ide $pycharm_version
  echo PyCharm installation complete
fi

if [ $configure_pycharm_launcher == true ]
then
  echo Reconfiguring PyCharm launcher to work with ROS...
  # Launch PyCharm usin bash after loading the virtual environment
  sed -i 's/Exec="/Exec=bash -i -c "useoars \&\& /' $HOME/.local/share/applications/jetbrains-pycharm*.desktop
  echo PyCharm launcher configuration complete
fi

echo \n\n
echo "Setup complete. Please run the following line to make ROS available in the current terminal window:"
echo
echo "   source ~/.bashrc && source $venv_activate"
echo
echo "Any time you open a new terminal and would like to run ROS commands in it, run the following:"
echo
echo "   useoars"
echo
echo "Enjoy!"
