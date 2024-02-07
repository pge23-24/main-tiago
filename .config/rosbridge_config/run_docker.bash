#!/bin/bash

# Lancement du docker

docker run -it --rm \
    --privileged \
    --net host \
    --ipc host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    cario360/rosbridge
