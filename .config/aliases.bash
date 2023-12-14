TIAGO_SSID="PMB2-46c"
TIAGO_PWD="P@L-R0b0t1cs"

package-link() {
    echo "[ROS] Adding your packages to workspace ..."
    find /packages/ \
        -maxdepth 1 \
        -mindepth 1 \
        -type d \
        -exec ln -sf '{}' /pal_mobile_base_ws/src/ \;
}

tiago-connect () {
    echo "[NET] Connecting to $TIAGO_SSID ..."
    nmcli device wifi connect $TIAGO_SSID password $TIAGO_PWD
    export ROS_MASTER_URI="http://10.68.0.1:11311"
    export ROS_IP=$(hostname -I | cut -d' ' -f1)
    echo "[TIAGO] Connected to ROS master"
}

tiago-disconnect () {
    export ROS_MASTER_URI="http://localhost:11311"
    unset ROS_IP
    echo "[TIAGO] Disconnected from ROS master"
}

alias rviz='rosrun rviz rviz'
alias gazebo='tiago-disconnect; roslaunch pmb2_gazebo pmb2_gazebo.launch'