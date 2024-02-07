#!/bin/bash

# Charger l'environnement ROS
source /opt/ros/foxy/setup.bash

# Construire le package spécifié
colcon build --packages-select py_pubsub_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select py_pubsub --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3


source install/setup.bash