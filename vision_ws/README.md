# VISION_WS

## Utilisation

### Build

```bash
source scripts/build_vision_ws.sh
```

### Lancement

#### Traitement Yolo toutes les caméras

```bash
bash scripts/run_all_camera_yolo.sh
# ou
ros2 launch py_pubsub launch_all_camera.py
```

#### Display toutes les caméras

```bash
bash scripts/run_all_display.sh
# ou
ros2 launch py_pubsub launch_all_camera.py
```

#### Traitement Yolo sur une caméra

```bash
bash scripts/run_camera_yolo.sh <id>
ros2 run py_pubsub camera --ros-args -p image_topic_id:=<id> 
```

#### Display une caméra

```bash
bash scripts/run_display.sh <id>
ros2 run py_pubsub display --ros-args -p image_topic_id:=<id> 
```

## Prototype

Activer la venv :

```bash
source .venv/bin/activate
```

Désactiver la venv :

```bash
deactivate
```
