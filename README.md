# turtlesim_drawer

Node for drawing shapes using the trace left by turtlesim.

### Synopsis

This package provides a node for drawing geometric shapes using the trace left by turtlesim.
Additionally, a second node is provided to control the drawer from within a web browser.

### Dependencies
* [ros](https://github.com/ros) - ROS core stack
* [catkin](http://www.ros.org/wiki/catkin) - cmake and Python based buildsystem
* [ros_tutorials](https://github.com/ros/ros_tutorials) - needed for turtlesim
* [roslibjs](https://github.com/RobotWebTools/roslibjs) - needed for the controller node
* [flask](https://github.com/pallets/flask) - needed for the controller node

### Install

Install dependencies:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    sudo apt-get update
    sudo apt-get -y install ros-kinetic-ros-base ros-kinetic-ros-tutorials ros-kinetic-rosbridge-suite python-flask

Setup ROS environment:

    source /opt/ros/kinetic/setup.bash

Create a catkin workspace as per the instructions in http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Get the source code:

    cd ~/catkin_ws/src/
    git clone https://github.com/dmnunez1993/turtlesim_drawer.git

Install:

    cd ~/catkin_ws
    catkin_make install

### Run

Reload the environment (For turtlesim_drawer msgs and srvs):

    source ~/catkin_ws/devel/setup.bash

Launch:

    roslaunch turtlesim_drawer turtlesim_drawer.launch

From a web browser visit http://localhost:12000

Enjoy!!
