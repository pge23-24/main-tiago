#!/bin/bash

start_time=$(date +%s)

BLUE='\033[0;34m' # Blue
NC='\033[0m' # No Color

# verification de l'installation de ROS
if rosversion -d | grep -q "Noetic"
then
    echo -e "${BLUE}ROS Noetic est déjà installé${NC}"
else
    echo -e "${BLUE}Installation de ROS Noetic${NC}"
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
rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall

#installations 

# Check if rosdep is installed
if ! command -v rosdep &> /dev/null
then
    echo -e "${BLUE}rosdep could not be found${NC}"
    echo -e "${BLUE}Installing rosdep...${NC}"
    sudo apt-get install python-rosdep -y
else
    echo -e "${BLUE}rosdep is installed${NC}"
fi

# rosdep init and update
sudo rosdep init
rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

echo -e "${BLUE}Installation des packages suplémentaires...${NC}"
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-four-wheel-steering-controller
sudo apt-get install ros-noetic-urdf-geometry-parser
sudo apt-get install ros-noetic-ddynamic-reconfigure
sudo apt-get install ros-noetic-people

echo -e "${BLUE}Installation de pre-commit...${NC}"
pip3 install pre-commit
pre-commit install

echo -e "${BLUE}Build du workspace...${NC}"

if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc
then
    echo -e "${BLUE}ROS is not in bashrc${NC}"
    echo -e "${BLUE}Adding ROS to bashrc...${NC}"
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
else
    echo -e "${BLUE}ROS is in bashrc${NC}"
fi

catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2) -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash

echo -e "${BLUE}Nettoyage...${NC}"
rm  tiago_public-noetic.rosinstall
rm src/.rosinstall

# End timing
end_time=$(date +%s)

# Calculate execution time
minutes=$(expr $execution_time / 60)
seconds=$(expr $execution_time % 60)

echo -e "${BLUE}Installation terminée en $minutes minutes et $seconds secondes${NC}"
