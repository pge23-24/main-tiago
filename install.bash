#!/bin/bash

SECONDS=0

BLUE='\033[0;34m' # Blue
NC='\033[0m' # No Color

echo -e "${BLUE}Mise à jour... ${NC}"
sudo apt update
sudo apt -y upgrade

# verification de l'installation de ROS
if rosversion -d | grep -q "Noetic"
then
    echo -e "${BLUE}ROS Noetic est déjà installé${NC}"
else
    echo -e "${BLUE}Installation de ROS Noetic${NC}"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    sudo apt update
    sudo apt install -y ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
fi

# installation des packages pal et ros
mkdir -p src

echo -e "${BLUE}Installation des packages Pal Robotics et des dépendances...${NC}"
# vérifification de l'installation de wget
if ! command -v wget &> /dev/null
then
    echo -e "${BLUE}wget could not be found${NC}"
    echo -e "${BLUE}Installing wget...${NC}"
    sudo apt-get install wget -y
else
    echo -e "${BLUE}wget is installed${NC}"
fi


source /opt/ros/noetic/setup.bash
wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall


sudo apt install python3-rosinstall -y

rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall 

#installations 

# Check if rosdep is installed
if ! command -v rosdep &> /dev/null
then
    echo -e "${BLUE}rosdep could not be found${NC}"
    echo -e "${BLUE}Installing rosdep...${NC}"
    sudo apt install python3-rosdep2 -y
else
    echo -e "${BLUE}rosdep is installed${NC}"
fi

# rosdep init and update
sudo rosdep init
rosdep update 

rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

echo -e "${BLUE}Installation des packages suplémentaires...${NC}"

#sudo apt-get install -y ros-noetic-navigation \
#                        ros-noetic-map-server \
#                        ros-noetic-four-wheel-steering-controller \
#                        ros-noetic-urdf-geometry-parser \
#                        ros-noetic-ddynamic-reconfigure \
#                        ros-noetic-people \
#                        ros-noetic-control-toolbox \
#                        ros-noetic-polled-camera \
#                        ros-noetic-camera-info-manager \
#                        ros-noetic-xacro \
#                        ros-noetic-tf-conversions \
#                        ros-noetic-robot \
#                        ros-noetic-moveit \
#                       ros-noetic-moveit \
#                       ros-noetic-pcl-ros


echo -e "${BLUE}Installation de pre-commit...${NC}"
# Check if pip3 is installed
if ! command -v pip3 &> /dev/null
then
    echo -e "${BLUE}pip3 could not be found${NC}"
    echo -e "${BLUE}Installing pip3...${NC}"
    sudo apt-get install python3-pip -y
else
    echo -e "${BLUE}pip3 is installed${NC}"
fi

pip3 install pre-commit 
pre-commit install



echo -e "${BLUE}Build du workspace...${NC}"

if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc
then
    echo -e "${BLUE}ROS is not in bashrc${NC}"
    echo -e "${BLUE}Adding ROS to bashrc...${NC}"
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
else
    echo -e "${BLUE}ROS is in bashrc${NC}"
fi


if ! command -v catkin &> /dev/null
then
    echo -e "${BLUE}catkin could not be found${NC}"
    echo -e "${BLUE}Installing catkin...${NC}"
    sudo apt install python3-catkin-tools python3-osrf-pycommon -y
else
    echo -e "${BLUE}catkin is installed${NC}"
fi

echo -e "${BLUE}Mise à jour (2)... ${NC}"
sudo apt update
sudo apt upgrade -y

catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2) -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

echo -e "${BLUE}Nettoyage...${NC}"
rm tiago_public-noetic.rosinstall*
rm src/.rosinstall*

# End timing
duration=$SECONDS

echo -e "${BLUE}Installation terminée en $(($duration / 60)) minutes et $(($duration % 60)) secondes${NC}"
