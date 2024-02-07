#!/bin/bash

# source workspace (lancer script avec `source` pour que les variables d'environnement soient conservées)
source install/setup.bash

# Simuler l'arrivé d'images brutes sur le topic /Cam1/image_raw avec une image de cario
ros2 run py_pubsub simu_ecal