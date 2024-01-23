#!/bin/bash

# Lancement du docker

docker run -it --rm \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri/card0:/dev/dri/card0 \
    -v /var/run/dbus:/var/run/dbus \
    --privileged \
    --net host \
    --ipc host \
    -v ./ros1_ws/src:/workspace/src/pge_packages \
    -e ROS_MASTER_URI=http://localhost:11311 \
    cario360/rosbridge
