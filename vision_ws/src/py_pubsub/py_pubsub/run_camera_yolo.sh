#!/bin/bash

# Charger l'environnement ROS
source /opt/ros/foxy/setup.bash

source install/setup.bash

# Exécuter le noeud ROS avec l'argument de la caméra passé au script
cam_id=$1
echo "$cam_id"
# Exécuter camera_yolo avec l'identifiant de la caméra
ros2 run py_pubsub camera_yolo --cam $cam_id