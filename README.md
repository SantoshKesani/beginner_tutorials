# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Author
Santosh Kesani

## Overview
The current package is built for the demonstration of simple publisher and subscriber nodes. An addition is the demonstration of a service and launch file. The listener.cpp file is the subscriber and the talker.cpp is the publisher. The talker file publishes a simple string and this is heard by the listener file. The laucnh file is located in launch folder and service is create in srv folder. Service file is generated to make changes to the default message published by the talker file. Launch file is created to run both talker and listener at the same time.

## Dependencies
The package has the following dependencies:

- catkin_ws
- roscpp
- std_msgs
- message_generation

## Build method
For the package, you must create and build the catkin workspace as follows in the command line:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
cd src/
```
Now clone the repo into the src folder
```
git clone --recursive https://github.com/SantoshKesani/beginner_tutorials.git
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash
```
## Running the Publisher and Subscriber
After building the package in a catkin workspace, run the following commands from root workspace(source the workspace if any command fails):
```
In Terminal-1: roscore
In Terminal-2: rosrun beginner_tutorials talker
In Terminal-3: rosrun beginner_tutorials listener
```
## Running the Launch file and service
Follow the following commands on console to run the launch file and the service(Make sure you source the workspace):
```
In Terminal-1: roscore
In Terminal-2: roslaunch beginner_tutorials tlnode.launch set_freq:="set the loop rate(int)"
In Terminal-3: rosservice call /new_string "Your Message"
```
## To check the rqt console log
Run the command: rqt_console in a new terminal after running the launch file to see the log
