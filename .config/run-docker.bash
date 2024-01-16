#!/bin/bash

# Lancement du docker
xhost +local:root

# HOW IT WORKS
# allow access to network manager - dbus, net-host, privileged
# ROS shenanigans - net-host, ipc-host 
# Display sharing - e-display, x11-unix, dri-card0 

docker run -it --rm \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri/card0:/dev/dri/card0 \
    -v /var/run/dbus:/var/run/dbus \
    --privileged \
    --net host \
    --ipc host \
    -v ./src:/workspace/src/pge_packages \
    pge/tiago-rob:latest

xhost -local:root