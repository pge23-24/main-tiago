#!/bin/bash

# Lancement du docker
xhost +local:root

# HOW IT WORKS
# allow access to network manager - dbus, net-host, privileged
# ROS shenanigans - net-host, ipc-host, add-host
# Display sharing - e-display, x11-unix, dri-card0

docker run -it --rm \
    -e DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri/card0:/dev/dri/card0 \
    -v /var/run/dbus:/var/run/dbus \
    --add-host pmb2-46c:10.68.0.1 \
    --privileged \
    --net host \
    --ipc host \
    --pid host \
    -v ./rob_ws/src:/pal_mobile_base_ws/src/pge_packages \
    cario360/ros-tiago:latest

xhost -local:root
