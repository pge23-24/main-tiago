#!/bin/bash


# ROS export
export ROS_MASTER_URI=http://10.68.0.1:11311
export ROS_IP=10.68.0.132

echo -e "\033[1;94m Building vision_ws... \033[0m"
source vision_ws/scripts/build_vision_ws.sh
echo -e "\033[1;94m Launching ecal gateway... \033[0m"
source vision_ws/scripts/run_ecal_gateway.sh &
cd ~/rob2-tiago/
echo -e "\033[1;94m Starting vision nodes... \033[0m"
source vision_ws/scripts/run_all_camera_yolo.sh 