# VISION_WS

Cette documentation ne prent pas en compte Docker

## Build

Premier build :

```bash
colcon build --packages-select py_pubsub_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Ensuite, celle ci suffit : 

```bash
colcon build --packages-select py_pubsub_msgs
```

## Utilisation

Activer la venv :
```bash
source .venv/bin/activate
```

Désactiver la venv :
```bash
deactivate
```

Lancement script traitement caméra (entre 1 et 4):
```bash
bash run_camera_yolo.sh <num_cam>
```

Lancement script display caméra (entre 1 et 4):
```bash
bash run_display.sh <num_cam>
```

## Communication 

Les messages sont : 
* angles et classes + header : py_pubsub_msgs.msg ClassCoordinates
* l'image au format standard ROS : sensor_msgs.msg Image

# Charger l'environnement ROS
source /opt/ros/foxy/setup.bash

# Construire le package spécifié
```bash
colcon build --packages-select py_pubsub_msgs --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon build --packages-select py_pubsub --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```
