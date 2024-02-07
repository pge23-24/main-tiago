#!/bin/bash

# source workspace (lancer script avec `source` pour que les variables d'environnement soient conservées)
source install/setup.bash

# Exécuter le noeud ROS avec l'argument de la caméra passé au script
cam_id=${1:-1}
echo "Cam : $cam_id"

yolo_version=${2:-v8}
echo "Yolo : $yolo_version"

tracker_enabled=${3:-False}
echo "Tracker : $tracker_enabled"

# Exécuter camera_yolo avec l'identifiant de la caméra
ros2 run py_pubsub camera_yolo --cam $cam_id --yolo $yolo_version --tracker $tracker_enabled