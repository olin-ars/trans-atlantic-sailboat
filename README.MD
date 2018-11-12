# The First Autonomous Trans-Atlantic Sailboat

To this day, no team has successfully crossed the Atlantic Ocean in an autonomous sailboat. Teams such as
[UBC's Transatlantic Challenge](http://ubctransat.com/) team and [SCOUT Transatlantic](http://gotransat.com/) have made
it close, but the coast of Western Europe remains elusive to the robotic seafarers. The exceedingly tumultuous
conditions of the North Atlantic are proving to be a fierce opponent, requiring some of the best engineers in the world
to unite in order to be conquered.

# Setting up a Dev Environment

In order to get a new development machine up and running, simply do the following
on a computer running Ubuntu 16.04:

  1) Open a terminal and run `mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`
  2) Clone the repo by running `git clone https://github.com/olin-robotic-sailing/trans-atlantic-sailboat.git`
  3) Run `cd trans-atlantic-sailboat && ./setup` to run the setup script
  4) Run the command it says to run afterwards

The setup script will walk you through installing ROS and setting it up with Python
in a way that will not conflict with other Python configurations (e.g. Anaconda).
It will also give you the option to install the PyCharm integrated development
environment, which is great for debugging, and configure it for use with ROS.

Just enter your password when it asks for it and press Enter when it asks for the
PyCharm installation path (this will accept the default).

# Current State

Status updates can be found on our [Medium blog](http://medium.com/oars).

## Architecture Overview

![OARS architecture overview](https://imgur.com/zAlyEsM.png)

## Repository Overview

![OARS code repository structure](https://imgur.com/52Hw9D2.png)

## Electrical System Overview

![OARS electrical system overview](https://imgur.com/eauzcEs.png)

Our boat is controlled by a Raspberry Pi running ROS Kinetic. We actuate the sails and rudder using Robotis Dynamixel
servos (MX-64s and an AX-12, respectively). We also use an Airmar 200WX ultrasonic weather station for wind speed and
direction, heading, and position.

## About OARS
<img src="https://imgur.com/Y1towMB.png" alt="OARS logo" width="30%" align="right">

[Olin Aquatic Robotic Systems](https://www.olinaquabots.com/) (OARS) is a build team at [Olin College of Engineering](http://www.olin.edu/)
in Needham, MA. We've been around in various forms since 2013, designing autonomous sailboats as well as boats for the
[AUVSI RoboBoat competition](http://www.robonation.org/competition/roboboat).
