#!/bin/bash

# Connexion au routeur du wi-fi
SSID=PMB2-46c
PWD=P@L-R0b0t1cs
nmcli device wifi connect "$SSID" password "$PWD"

# Chemin vers le setup.bash de ROS
echo "Sourcing ROS Noetic"
source /opt/ros/noetic/setup.bash

# Chemin vers le setup.bash du projet (assurez-vous de remplacer <chemin_vers_devel> par le chemin correct)
echo "Sourcing Workspace"
source devel/setup.bash

# Configuration de l'URI du maître ROS
echo "Settuping ROS_MASTER_URI"
export ROS_MASTER_URI=http://10.68.0.1:11311

# Récupération de l'adresse IP du PC sur le réseau
echo "Settuping ROS_IP	"
export ROS_IP=$(hostname -I | cut -d' ' -f1)

