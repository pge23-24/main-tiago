# Ubuntu setup

1. Install docker

Run the following command then restart computer (code viewable [here](https://gist.github.com/Guillaume-prog/e9551041f0637e4452c93c98dd96f85b))

```bash
curl -s https://gist.githubusercontent.com/Guillaume-prog/e9551041f0637e4452c93c98dd96f85b/raw/docker-install.bash?_=$(uuidgen) \ | sudo bash
```

2. Download the docker image

```bash
docker pull frenchfry06/tiago-rob:latest
docker tag frenchfry06/tiago-rob:latest pge/tiago-rob:latest 
```

### Usage

To run the docker : 
```bash
./setup/run-docker.bash
```

# Aliases

- `tiago-connect`: connects container to robot network and sets necessary env variables
- `tiago-disconnect`: resets env variables. You are not disconnected from the robot's network
- `rviz`: runs an empty rviz window
- `gazebo`: runs gazebo simulation. Automatically disconnects from robot to avoid issues