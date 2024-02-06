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

RUN apt install -y g++ cmake make git libgtk2.0-dev pkg-config
RUN git clone https://github.com/opencv/opencv.git
RUN mkdir -p build && cd build \
    && cmake ../opencv \
    && make -j4 \
    && make install

RUN apt install -y ros-foxy-cv-bridge \
                sudo

# Copy Conti Gateway
COPY Fwk_licensed /Fwk_licensed
COPY CES_FWK_Gateway_ecalToRos2 /CES_FWK_Gateway_ecalToRos2

# install dependencies of Gateway
# Clone eCAL
RUN git clone --recurse-submodules https://github.com/continental/ecal.git

# Install necessary packages for eCAL (like protobuf)
RUN apt-get update && apt-get install -y \
    libprotobuf-dev \
    protobuf-compiler \
    libprotoc-dev \
    libtclap-dev \
    libhdf5-serial-dev \
    libcurl4-openssl-dev \
    libasio-dev \
    libgtest-dev \
    libtinyxml2-dev \
    libjsoncpp-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    libasio-dev \
    libgtest-dev \
    libtinyxml2-dev \
    libjsoncpp-dev \
    libssl-dev

# Install Qt5 with additional packages
RUN apt-get update && apt-get install -y \
    qt5-default \
    libqwt-qt5-dev 


# Build eCAL
RUN mkdir -p ecal/build && cd ecal/build \
    && cmake .. \
    && make -j$(nproc) \
    && make install

# Gateway ROS dependencies
# Install tf2_msgs
RUN apt-get update && apt-get install -y \
    ros-foxy-tf2-msgs \
    ros-foxy-image-transport \
    ros-foxy-tf2

# Build Gateway
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash \
    && cd /CES_FWK_Gateway_ecalToRos2 \
    && colcon build"

# Other requirements
RUN apt-get update && apt-get install -y \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-rviz2 \
    freeglut3

RUN pip install numpy==1.24.4 \
                lapx==0.5.2

# Add shortcuts and aliases
RUN echo "cd /workspace" >> /root/.bashrc
COPY .config/ros2_config/aliases.bash /root/.bash_aliases

RUN pip install gitpython==3.1.30 \
                setuptools==65.5.1

RUN apt-get update && apt-get install -y \
    inetutils-ping