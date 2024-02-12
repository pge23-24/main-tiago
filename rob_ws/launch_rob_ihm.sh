#!/bin/bash

export ROS_MASTER_URI=http://10.68.0.1:11311
export ROS_IP=10.68.0.132

echo -e "\033[1;94m Launch ihm agent... \033[0m"
rosrun cario_ihm_agent relay_controller