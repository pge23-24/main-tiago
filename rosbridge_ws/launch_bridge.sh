#!/bin/bash

# Sourcing for ROS2 and ROS1
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
# Sourcing workspaces
source rob_ws/devel/setup.bash
source vision_ws/install/setup.bash

# export ros1 variables
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.132
# export ros2 variables
export ROS_HOSTNAME=$ROS_IP
export IGN_IP=127.0.0.1

# run ros1 bridge
ros2 run ros1_bridge dynamic_bridge

