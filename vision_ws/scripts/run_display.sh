#!/bin/bash

# Exécuter le noeud ROS avec l'argument de la caméra passé au script
cam_id=${1:-1}
topic_name="annotated_images_$cam_id"

# Exécuter display avec l'identifiant de la caméra
ros2 run py_pubsub display --topic "$topic_name"