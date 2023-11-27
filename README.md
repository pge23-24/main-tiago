# main-tiago
Repo principal pour le code qui sera sur le robot

# GIT ET L'INTEGRATION CONTINUE

## Utilisation de Git lors du PGE

Le code source du projet est sur un repo principal, qui sert de référence de l'avancée du projet. Chaque équipe a un repo forké du repo principal, et c'est sur ce repo que les équipes travaillent. Lorsqu'une fonctionnalité est terminée, elle est pushée sur le repo de l'équipe, puis un pull request est fait sur le repo principal. Un membre de l'équipe de validation du code se charge de vérifier le code et de le merger sur le repo principal.

Pour assurer la "qualité" du code produit et éviter les problèmes d'intégration, plusieurs outils plus ou moins automatisés sont utilisés :

- **cpplint** : vérifie que le code c++ est conforme aux normes de codage de Google (exigence par le client)
Cpplint est executé automatiquement avant chaque commit. Si des erreurs sont détectées, le commit est annulé et un message d'erreur est affiché. Il est donc nécessaire de corriger les erreurs avant de pouvoir commit. Il ne vérifie pas les erreurs de compilation, uniquement les erreurs de style.

- **Google Test** : framework de test unitaire pour c++
Les tests unitaires seront tous executé avant de merge une pull request sur le repo principal. Le taux de couverture de ces tests sera vérifié.
La compilation sera également vérifiée à ce moment là.

## Cpplint avec Pre commit

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

L'utilisation d'un environement conda est recommandé.

source le l'environnement ROS :

```bash
source /opt/ros/noetic/setup.bash
```

build normal d'un workspace ROS, à executer dans le workspace :

```bash
catkin build
```

Des problèmes de librairies python peuvent survenir. En cas de problème de librairies manquante au fonctionnement de catkin (ex: empy) (seulement la première fois, après un build normal suffit):

```bash
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
```

L'installation de pre-commit :

```bash
pip3 install pre-commit
```

Configuration de pre-commit :

```bash
pre-commit install
```

# Procédure de connexion au robot simulé

# Procédure de connexion au robot Tiago
## Dépendances
- Ubuntu 20
- ROS Noetic : http://wiki.ros.org/noetic/Installation/Ubuntu
- PAL packages :
    - Suivre ce tutoriel : http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS
    - Avant la dernière commande de l'étape 3, exécuter ces lignes :
      ```
      sudo apt-get install ros-noetic-navigation
      sudo apt-get install ros-noetic-map-server
      sudo apt-get install ros-noetic-four-wheel-steering-controller
      sudo apt-get install ros-noetic-urdf-geometry-parser
      sudo apt-get install ros-noetic-ddynamic-reconfigure
      sudo apt-get install ros-noetic-people
      sudo apt-get install ros-noetic-twist-mux
      ```

## Connexion
### Méthode 1
Utiliser la commande suivante dans le workspace :
```
source connect_to_robot.bash
```

### Méthode 2
#### Etape 1
Se déplacer dans le workspace d'intérêt <ros_ws> :
```
cd <ros_ws>
```

#### Etape 2
Dans le workspace, mettre à jour les paquets :
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

#### Etape 3
Se connecter au réseau Wi-Fi émis par le robot TiaGo :
```
SSID : PMB2-46c
PWD  : P@L-R0b0t1cs
```

#### Etape 4
Récupérer l'adresse IP du PC sur le nouveau réseau :
```
ip address
```

#### Etape 5
Mettre à jour les variables d'environnement :
```
export ROS_MASTER_URI=http://10.0.68.1:11311
export ROS_IP=<pc_ip>
```