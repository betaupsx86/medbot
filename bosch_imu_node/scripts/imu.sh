#!/bin/bash
sudo su <<HERE
source /opt/ros/indigo/setup.bash
source /home/mario/medbot_ws/devel/setup.bash
roslaunch bosch_imu_node imu.launch
HERE
