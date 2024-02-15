
#!/bin/bash

# Sourcing for ROS2 and ROS1
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
# Sourcing workspaces
source rob_ws/devel/setup.bash
source vision_ws/install/setup.bash

echo -e "\033[1;94m Building ros1 bridge... \033[0m"
cd rosbridge_ws && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
echo -e "\033[1;94m Build ros1 bridge complete ! \033[0m"