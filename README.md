# turtlesim_drawer

Node for drawing shapes using the trace left by turtlesim.

### Synopsis

This package provides a node for drawing geometric shapes using the trace left by turtlesim.
Additionally, a second node is provided to control the drawer from a web browser.

### Dependencies
* [catkin](http://www.ros.org/wiki/catkin) - cmake and Python based buildsystem
* [ros_tutorials](https://github.com/ros/ros_tutorials) - needed for turtlesim
* [flask](https://github.com/pallets/flask) - needed for the controller node

### Install

Install dependencies:

    sudo apt-get -y install ros-kinetic-ros-base ros-kinetic-ros-tutorials ros-kinetic-rosbridge-suite python-flask

Setup ROS environment:

    source /opt/ros/kinetic/setup.bash

Create a catkin workspace as per the instructions in http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Get the code:

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

From a web broser visit http://localhost:12000

Enjoy!!
