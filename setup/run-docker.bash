#!/bin/bash

# Lancement du docker
xhost +local:root

docker run -it \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/dri/card0:/dev/dri/card0 \
    -p 11311:11311 \
    pge/tiago-rob:latest

xhost -local:root