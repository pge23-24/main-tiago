#!/bin/bash


# ROS export
export ROS_MASTER_URI=http://10.68.0.1:11311
export ROS_IP=10.68.0.132

echo -e "\033[1;94m Launching ecal gateway... \033[0m"
source vision_ws/scripts/run_ecal_gateway.sh &

echo -e "\033[1;94m Starting vision nodes... \033[0m"
source vision_ws/scripts/run_all_camera_yolo.sh 
