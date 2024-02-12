# SETUP

## install
pull
voir CONTROL_ROBOT pour setup tiago base

## build
dans un nouveau terminal :
- source que ROS Noetic
- build rob-ws

dans un nouveau terminal :
- source que ROS Foxy
- build vis-ws

dans un nouveau terminal :
- source Noetic, Foxy, rob-ws, vis-ws
- build bridge

## run
### ROS1
dans un nouveau terminal :
- source que ROS Noetic
- source rob-ws
- export ROS_MASTER_URI=<IP du robot:port du robot>
- export ROS_IP=<IP du PC>
- launch your nodes

### Bridge
dans un nouveau terminal :
- source Noetic, Foxy, rob-ws, vis-ws
- source bridge
- export ROS_MASTER_URI=<IP du robot:port du robot>
- export ROS_IP=<IP du PC>
- export ROS_HOSTNAME=<IP du PC>
- export IGN_IP=127.0.0.1
- run bridge

### ROS2
dans un nouveau terminal :
- source que ROS Foxy
- source vis-ws
- export ROS_MASTER_URI=<IP du robot:port du robot>
- export ROS_IP=<IP du PC>
- launch your nodes




