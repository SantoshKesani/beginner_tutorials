# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)


## Author
Santosh Kesani

## Overview
The current package is built for the demonstration of simple publisher and subscriber nodes. The listener.cpp file is the subscriber and the talker.cpp is the publisher. The talker file publishes a simple string and this is heard by the listener file.

## Dependencies
The package has the following dependencies:

- catkin_ws
- roscpp
- std_msgs

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
