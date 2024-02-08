#!/bin/bash

# source workspace (lancer script avec `source` pour que les variables d'environnement soient conservées)
source install/setup.bash

# Exécuter le noeud ROS avec l'argument de la caméra passé au script
cam_id=${1:-1}
echo "Cam : $cam_id"

# Exécuter camera_yolo avec l'identifiant de la caméra
ros2 run py_pubsub camera_yolo --ros-args -p cam_id:=$cam_id
