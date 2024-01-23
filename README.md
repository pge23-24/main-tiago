# main-tiago

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Repo principal pour le code qui tourne sur le PC du robot Tiago.

## Installation

### Prérequis

Ubuntu 20.04
Git installé :

```bash
sudo apt install git
```

Clé SSH configurée pour Github (<https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh>)

### Clonage et Docker

Cloner le repo :

```bash
git clone <repo>
```

L'environement de travail est dans des Docker. Il y a un docker pour ROS1, un pour ROS2 et un pour ROS bridge.
Le détail de l'installation de Docker est dans le fichier README_CONFIG.md, ainsi que l'utilisation des Docker.

L'installation de pre-commit :

```bash
pip3 install pre-commit
pre-commit install
```

## Procédure de connexion au robot simulé

## Procédure de connexion au robot Tiago

Pour ce connecter au robot, il faut d'abord se connecter au réseau Wi-Fi émis par le robot, puis indiquer à ROS1 de se connecter au master du robot. Le script suivant le fait automatiquement :
Utiliser la commande suivante dans le workspace :

```bash
source connect_to_robot.bash
```

Pour se connecter "manuellement" :

```bash
SSID : PMB2-46c
PWD  : P@L-R0b0t1cs
```

Variables d'environnement à configurer pour ROS :

```bash
export ROS_MASTER_URI=http://10.0.68.1:11311
export ROS_IP=<pc_ip>
```

## GIT ET L'INTEGRATION CONTINUE

### Utilisation de Git lors du PGE

Le code source du projet est sur un repo principal, qui sert de référence de l'avancée du projet. Chaque équipe a un repo forké du repo principal, et c'est sur ce repo que les équipes travaillent. Lorsqu'une fonctionnalité est terminée, elle est push sur le repo de l'équipe, puis un pull request est fait sur le repo principal. Un membre de l'équipe de validation du code se charge de vérifier le code et de le merger sur le repo principal.

Pour assurer la "qualité" du code produit et éviter les problèmes d'intégration, plusieurs outils plus ou moins automatisés sont utilisés :

- **cpplint** : vérifie que le code c++ est conforme aux normes de codage de Google (exigence par le client)
Cpplint est executé automatiquement avant chaque commit. Si des erreurs sont détectées, le commit est annulé et un message d'erreur est affiché. Il est donc nécessaire de corriger les erreurs avant de pouvoir commit. Il ne vérifie pas les erreurs de compilation, uniquement les erreurs de style.

- **Google Test** : framework de test unitaire pour c++
Les tests unitaires seront tous executé avant de merge une pull request sur le repo principal. Le taux de couverture de ces tests sera vérifié.
La compilation sera également vérifiée à ce moment là.
