#!/bin/bash

# source workspace (lancer script avec `source` pour que les variables d'environnement soient conservées)
source install/setup.bash

# Exécuter le noeud ROS avec l'argument de la caméra passé au script
topic_id=${1:-1}

# Exécuter display avec l'identifiant de la caméra
ros2 run py_pubsub display --ros-args -p image_topic_id:=$topic_id
