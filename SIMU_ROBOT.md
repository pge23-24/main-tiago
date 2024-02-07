# Simulation du robot 

## Lancement de la navigation de base

Sur le docker, ouvrir un terminator et le split en deux. Sur l'un d'eux lancer un roscore : 

```bash
roscore
```

Sur l'autre sourcer le devel puis lancer la commande roslaunch :

```bash
source devel/setup.bash
roslaunch pmb2_2dnav_gazebo pmb2_navigation.launch public_sim:=true
```
