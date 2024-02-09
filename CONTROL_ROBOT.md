# Controle du robot Tiago

## Procédure de connexion au robot Tiago

[Doc temporaire](https://docs.google.com/document/d/1pG6wYegVpr1ERmZFlUUp6Ur5FBW4FbInwJ-JJSAe6z4/edit#heading=h.rqqvuok4lc4l)

Pour se connecter au robot, il faut d'abord se connecter au réseau Wi-Fi émis par le robot, puis indiquer à ROS1 de se connecter au master du robot.

Utiliser l'alias du docker suivant :

```bash
tiago-connect
```

Et pour se connecter :

```bash
tiago-disconect
```

Pour se connecter "manuellement" :

```bash
SSID : PMB2-46c
PWD  : P@L-R0b0t1cs
```

Variables d'environnement à configurer pour ROS :

```bash
export ROS_MASTER_URI=http://10.68.0.1:11311
export ROS_IP=<pc_ip>
```

## Visualisation

Voir la carte et la navigation du robot dans rviz :

```bash
rosrun rviz rviz -d `rospack find pmb2_2dnav`/config/rviz/navigation.rviz
```

Avec plus d'option, un gestionnaire de map, ect :

```bash
rosrun rviz rviz -d `rospack find pmb2_2dnav`/config/rviz/advanced_navigation.rviz
```

## Navigation

Change le mode du robot pour creer une nouvelle map:

```bash
rosservice call /pal_navigation_sm "input: 'MAP'"
```

Pour passer en mode localisation :

```bash
rosservice call /pal_navigation_sm "input: 'LOC'"
```

Enregistrement de la map :

```bash
rosrun map_server map_saver -f <map_name>
rosservice call /pal_map_manager/save_map "directory: ''"
```

Les cartes sont stockées dans `.pal/pmb2_maps/configurations/`

Récupération de la map :

```bash
scp -r pal@pmb2-46c:/home/pal/.pal/pmb2_maps/configurations/<map-name> /path/to/destination
```

Choisir la map active :

```bash
rosservice call /pal_map_manager/change_map "input: '<map_name>'"
```

### Evitement d'obstacles dynamiques
Se connecter au robot:
```
ssh pal@pmb2-46c
password:_____
```
Modifier le fichier `move_base.launch`
```
sudo vim /opt/pal/ferrum/share/pal_navigation_cfg_pmb2/launch/move_base.launch
```
Appuyer sur `i` pour rentrer en mode édition.
Ajouter la ligne suivante dans la balise `node` nommé *move_base* après les remap:
```
<remap from="odom"    to="$(arg odom)"/>
<remap from="cmd_vel" to="$(arg cmd_vel)"/>
```
```
<remap from="move_base/local_costmap/costmap" to="to_inflate_costmap"/>
```
Appuyer sur *echap* pour quiter le mode édition. taper *:wq!* puis *entrée* pour sauvgarder le fichier.

Exécuter la commande suivante dans le terminal :  
```bash
pal-restart move_base
```

## Déplacement

avec les touches clavier :

```bash
rosrun key_teleop key_teleop.py
```

autrement, utiliser la manette :

* Vérifier que le bouton au dos soit bien sur D
* pointer l’arrière du robot
* appuyer sur X pour activer la manette
