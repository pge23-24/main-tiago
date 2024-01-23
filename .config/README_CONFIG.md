# CONFIGURATION

[Docker Hub](https://hub.docker.com/u/cario360)

## Résumé

Nous travaillons avec docker pour tous avoir le même environnement de travail. Il y a un docker pour ROS1, un pour ROS2 et un pour ROS bridge.

Les commandes pour les lancer depuis le workspace :

ROS1 : `bash .config/ros1_config/run-docker.bash`

ROS2 : `bash .config/ros2_config/run-docker.bash`

ROS bridge : `bash .config/rosbridge_config/run_docker.bash`

## Installation de DOCKER

### Ubuntu

Install docker
Run the following command then restart computer (code viewable [here](https://gist.github.com/Guillaume-prog/e9551041f0637e4452c93c98dd96f85b))

```bash
curl -s https://gist.githubusercontent.com/Guillaume-prog/e9551041f0637e4452c93c98dd96f85b/raw/docker-install.bash?_=$(uuidgen) \ | sudo bash
```

### WSL

Utiliser Docker Engine

## Travailler avec Docker

Pour avoir plusieurs terminaux dans le même docker :

```bash
terminator
```

ou alors, efficace mais moins intuitif :

```bash
tmux
```

/!\ Lancer plusieurs fois le même docker run dans des terminaux différents lancera des docker completement séparé. La communication ROS devrait fonctionner, mais rien d'autre.

## ROS 1 avec DOCKER

### Installation ROS1

Download the docker image

```bash
docker pull cario360/ros-tiago
```

### Usage ROS1

To run the docker :

```bash
bash .config/ros1_config/run-docker.bash
```

### Build ROS1

Les packages sont tous dans le docker, dans le dossier pge_packages.
Se placer dans le package à build :

```bash
catkin build --this
```

### Aliases

- `tiago-connect`: connects container to robot network and sets necessary env variables
- `tiago-disconnect`: resets env variables. You are not disconnected from the robot's network
- `rviz`: runs an empty rviz window
- `gazebo`: runs gazebo simulation. Automatically disconnects from robot to avoid issues

## ROS 2 avec DOCKER

### Installation ROS2

Download the docker image

```bash
docker pull cario360/ros2
```

### Usage ROS2

To run the docker :

```bash
bash .config/ros2_config/run-docker.bash
```

## ROS BRIDGE avec DOCKER

### Installation ROS BRIDGE

Download the docker image

```bash
docker pull cario360/rosbridge
```

### Usage ROS BRIDGE

To run the docker :

```bash
bash .config/rosbridge_config/run_docker.bash
```

Rien d'autre n'est nécessaire, le docker est configuré pour lancer le rosbridge au démarrage.
