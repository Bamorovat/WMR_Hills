# wmr_robot_hills
## SOSCO's Mobile Robot ROS Node (WMR_Robot_Hills)
## Author: [Mohammad Hossein Bamorovat Abadi](https://bamorovat.wordpress.com/).

This code implemented with C++ programing Language.
This package implemented in **ROS environment**.
This Project use **UDP Communication Protocol** to tranfare data between computer and robot.

# ROS
We have tested **wmr_robot_hills in Ubuntu 18.04 with ROS Melodic and Ubuntu 16.04 with ROS Kinetic**. If you do not have already installed ROS in your computer, we recommend you to install the Full-Desktop version of ROS Melodic: (http://wiki.ros.org/melodic/Installation/Ubuntu).

# Installation
Download and copy the wmr_robot_hills file at to catkin workspace/src (**Note: Remove "-master" from the end of the file name**):

    cd catkin_ws/src

go to wmr_robot_hills pakage and run codes below at ubuntu terminal:

    cd catkin_ws/src/wmr_robot_hills
    mkdir build
    cd build
    cmake ..
    make
 
# Usage
Befor run the package, we should Knowing the new package :

    . ~/catkin_ws/devel/setup.bash
    
Run the package:

    rosrun wmr_robot_hills wmr_robot_hills
