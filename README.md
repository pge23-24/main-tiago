# main-tiago

Repo principal pour le code qui sera sur le robot

# INSTALLATION DE L'ENVIRONNEMENT VIRTUEL

Pour installer l'environnement virtuel nécessaire au projet, exécutez la commande suivante dans le dossier `prototyping` du dossier du projet :

```bash
bash ./setup/venv_install.sh
```

Afin de désinstaller la venv, la procédure est identique mais le script à lancer est `venv_uninstall.sh`, toujours dans le dossier `prototyping`.

```bash
bash ./setup/venv_uninstall.sh
```

Le but actuel de cette venv n'est pas de faire tourner le workspace ros, mais de pouvoir avoir un environnement commun aux prototypages.

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
