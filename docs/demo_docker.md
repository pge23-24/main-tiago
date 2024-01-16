# Demo Docker

1. Charger tous les packages ROS 1 dans le dossier `src`

2. Lancer ceci pour vous mettre dans l'environnement ROS1 :
```bash
bash .config/run-docker.bash
```

3. `pwd` devrait retourner `/workspace`. Tu trouveras tes packages dans `/workspace/src/pge_packages`

4. Exécuter pour tout compiler :
```bash
catkin build
```
- `--continue-on-failure` : Continue la compilation même si l'un des packages rate.
- `--this` : Compile uniquement le package dans lequel tu es situé.
- `<package-1> <package-2>` : compile uniquement les packages spécifiés.

5. Source le code compilé
```bash
source devel/setup.bash
```

6. Lancer les packages avec `rosrun`
