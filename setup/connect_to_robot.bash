#!/bin/bash

echo "Connecting to the robot's network ..."

# Connexion au routeur du wi-fi
SSID=PMB2-46c
PWD=P@L-R0b0t1cs
nmcli device wifi connect "$SSID" password "$PWD"


# Récupération de l'adresse IP du PC sur le réseau
ROS_IP=$(hostname -I | cut -d' ' -f1)

echo -e "\nRun this command in the docker terminal:"
echo -e "========================================\n"
echo -e "source /ros_ws/tiago_connect.bash $ROS_IP"
echo -e "\n========================================"