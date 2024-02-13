#!/bin/bash

ABS_PATH=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)

terminator --new-tab -x $ABS_PATH/vision_ws/launch_vision_yolo.sh && bash --rcfile <(echo ". ~/.bashrc && terminator --new-tab")
terminator --new-tab -x $ABS_PATH/rob_ws/launch_rob_planner.sh && bash --rcfile <(echo ". ~/.bashrc && terminator --new-tab") &
terminator --new-tab -x $ABS_PATH/rob_ws/launch_rob_ihm.sh && bash --rcfile <(echo ". ~/.bashrc && terminator --new-tab")  &     
terminator --new-tab -x $ABS_PATH/rosbridge_ws/launch_bridge.sh && bash --rcfile <(echo ". ~/.bashrc && terminator --new-tab") &

