#!/bin/bash

# Sourcing for ROS2 and ROS1
source /opt/ros/noetic/setup.bash

echo -e "\033[1;94m Building rob_ws... \033[0m"
cd rob_ws && catkin build multi_obstacles_tracker_msgs
catkin build
echo -e "\033[1;94m Build rob_ws complete ! \033[0m"

source rob_ws/devel/setup.bash
