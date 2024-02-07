#!/bin/bash

# Lancement du docker
xhost +local:root

docker run -it --rm \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri/card0:/dev/dri/card0 \
    -v /var/run/dbus:/var/run/dbus \
    --privileged \
    --net host \
    --ipc host \
    --gpus all \
    --device /dev/nvidia0 \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v ./vision_ws/src:/workspace/src \
    -v ./vision_ws/scripts:/workspace/scripts \
    -v ./vision_ws/models:/workspace/models \
    cario360/ros2

xhost -local:root