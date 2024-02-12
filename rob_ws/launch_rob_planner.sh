#!/bin/bash

# source workspace
source rob_ws/devel/setup.bash

# ROS export
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.132

echo -e "\033[1;94m Launch costmap inflater... \033[0m"
roslaunch multi_obstacles_tracker local_costmap_inflater
