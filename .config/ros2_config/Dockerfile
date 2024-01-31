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
    terminator \
    python3-pip


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



# Install dependencies of the project

## Python dependencies
RUN pip install ultralytics \
                opencv-python \
                pillow \
                pywavefront \
                transformers \
                accelerate \
                numpy \
                natsort



## C++ dependencies
#method 1
# install opencv for c++, latest version
#RUN apt update && apt install -y cmake g++ wget unzip
#RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
#RUN unzip opencv.zip

#method 2
RUN apt install -y g++ cmake make git libgtk2.0-dev pkg-config
RUN git clone https://github.com/opencv/opencv.git
RUN mkdir -p build && cd build \
    && cmake ../opencv \
    && make -j4 \
    && make install


# RUN mkdir -p build && cd build \
#    && cmake ../opencv-4.x \
#    && cmake --build .


## Install Ecal to Ros2 gateway
## To run on the Conti machine

## copy licenses and gateway
#COPY ~/Desktop/Fwk_licensed /ecalToRos2/Fwk_licensed
#COPY ~/Desktop/CES_FWK_Gateway_ecalToRos2 /ecalToRos2/CES_FWK_Gateway_ecalToRos2

# rebuild the gateway
#RUN . /opt/ros/foxy/setup.bash \
#    && cd /ecalToRos2/CES_FWK_Gateway_ecalToRos2/CES_FWK_GATEWAY_ECALTOROS2/ \
#    && colcon build
