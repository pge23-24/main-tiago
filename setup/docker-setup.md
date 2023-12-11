# Ubuntu setup

1. Install docker

```bash
apt remove -y docker docker-engine docker.io containerd runc

apt update
apt install -y ca-certificates curl gnupg

install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  tee /etc/apt/sources.list.d/docker.list > /dev/null

apt update
apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
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

To connect to the robot : 
```bash
./setup/connect_to_robot.bash
```

# Windows setup
