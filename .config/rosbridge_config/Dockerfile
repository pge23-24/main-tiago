ARG ROS1_WS=/ros1_ws
ARG ROS2_WS=/ros2_ws

# Dockerfile to bridge the ros1 and ros2 messages
#
# Build the ros1 messages
# ------------------------
FROM cario360/ros-tiago:latest as ros1_builder

WORKDIR /workspace
COPY /rob_ws/src /workspace/src/ros1_packages

RUN catkin config --install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic 

#
# Build the ros2 messages
# ------------------------
FROM cario360/ros2 as ros2_builder

# source to have all build tools
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bash_profile

WORKDIR /workspace

COPY /vision_ws/src /workspace/src/ros2_packages

#
# Build the ros1_bridge
# -----------------------
FROM cario360/ros2 as ros_bridge_builder
# source to have all build tools
SHELL ["/bin/bash", "-c"]

# install noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update \
  && apt-get install -y ros-noetic-ros-base \
  && rm -rf /var/lib/apt/lists/*



# Get the ros messages
COPY --from=ros1_builder /opt/ros/noetic /opt/ros/noetic
# get ros/tf for ros1 
RUN apt-get update && apt-get install -y \
  ros-noetic-tf



# Get the ros2 messages
COPY --from=ros2_builder /opt/ros/foxy /opt/ros/foxy
# get ros/tf2 for ros2
RUN apt-get update && apt-get install -y \
  ros-foxy-tf2



# Deps for the bridge
RUN apt-get update && apt-get install -y \
  # Test deps
  git \
  ros-foxy-demo-nodes-cpp \
  ros-foxy-launch-testing \
  ros-foxy-launch-testing-ament-cmake \
  ros-foxy-launch-testing-ros \
  ros-noetic-rospy-tutorials \
  ros-noetic-roscpp-tutorials \
  # Build deps
  libboost-filesystem-dev \
  libboost-math-dev \
  libboost-regex-dev \
# libboost-signals-dev \
  libboost-thread-dev \
  liblog4cxx-dev \
  && rm -rf /var/lib/apt/lists/*


# Set up the environment
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/ros/noetic/lib
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_VERSION=2
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.6/site-packages:/opt/ros/noetic/lib/python2.7/dist-packages
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PATH=/opt/ros/foxy/bin:/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig
ENV CMAKE_PREFIX_PATH=/opt/ros/foxy:/opt/ros/noetic

# Build the bridge
WORKDIR /workspace/ros_bridge_ws

RUN mkdir -p src && cd src \
&& git clone -b foxy https://github.com/ros2/ros1_bridge.git

RUN source /opt/ros/foxy/setup.bash \
 && colcon build --merge-install --packages-select ros1_bridge --cmake-force-configure --install-base /opt/ros/foxy

RUN echo "source /workspace/ros_bridge_ws/install/setup.bash" >> ~/.bash_profile

#
# Run the ros1_bridge
# ---------------------------
FROM cario360/ros2 as ros_bridge_runner

# source to have all build tools
#SHELL ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash"]
SHELL ["/bin/bash", "-c"]

# install noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update \
  && apt-get install -y ros-noetic-ros-base \
  && rm -rf /var/lib/apt/lists/*


# Copy the ros outputs
COPY --from=ros_bridge_builder /opt/ros/noetic /opt/ros/noetic

# Copy the ros2 outputs
COPY --from=ros_bridge_builder /opt/ros/foxy /opt/ros/foxy

# Deps for the bridge
RUN apt-get update && apt-get install -y \
  # Test deps
  ros-foxy-demo-nodes-cpp \
  ros-foxy-launch-testing \
  ros-foxy-launch-testing-ament-cmake \
  ros-foxy-launch-testing-ros \
  ros-noetic-rospy-tutorials \
  ros-noetic-roscpp-tutorials \
  # Build deps
  libboost-filesystem-dev \
  libboost-math-dev \
  libboost-regex-dev \
# libboost-signals-dev \
  libboost-thread-dev \
  liblog4cxx-dev \
  && rm -rf /var/lib/apt/lists/*

# Set up the environment
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib:/opt/ros/noetic/lib
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ENV CMAKE_PREFIX_PATH=/opt/ros/noetic:/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_VERSION=1
ENV ROS_LOCALHOST_ONLY=0
ENV ROS_PYTHON_VERSION=2
ENV PYTHONPATH=/opt/ros/noetic/lib/python2.7/dist-packages:/opt/ros/foxy/lib/python3.6/site-packages
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PATH=/opt/ros/noetic/bin:/opt/ros/foxy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig

# source bridge
#SHELL ["/bin/bash", "-c", "source /workspace/ros_bridge_ws/install/setup.bash"]

# Run the print-pairs command to check if things are working properly
RUN source /opt/ros/foxy/setup.bash \
 && ros2 run ros1_bridge dynamic_bridge --print-pairs

CMD ["bash", "-c", "ros2 run ros1_bridge dynamic_bridge --bridge-all-pairs"]
