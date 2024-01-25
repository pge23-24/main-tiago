# Use an existing docker image as a base
FROM ros:foxy-ros-core-focal

ARG DEBIAN_FRONTEND=noninteractive

# Install necessary tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    network-manager \
    tmux \
    python3-rosmsg \
    terminator


# Set the working directory
WORKDIR /workspace

# Run commands
RUN apt-get update

# add sourcing to bashrc
# RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bash_profile

# QT config
RUN echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc

RUN mkdir -m 700 -p /run/user/$(id -u)
RUN echo "export XDG_RUNTIME_DIR=/run/user/$(id -u)" >> ~/.bashrc