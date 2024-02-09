#!/bin/bash

# Charger l'environnement ROS
source install/setup.bash

# Construire le package spécifié
colcon build --packages-select py_pubsub --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select py_pubsub_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

# Exécuter le noeud ROS avec l'argument de la caméra passé au script
cam_id=${1:-1}
topic_name="annotated_images_$cam_id"

# Exécuter display avec l'identifiant de la caméra
ros2 run py_pubsub display --topic "$topic_name"