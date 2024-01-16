TIAGO_SSID="PMB2-46c"
TIAGO_PWD="P@L-R0b0t1cs"

tiago-connect () {
    local skip_wifi=false

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --no-wifi)
                skip_wifi=true
                shift
                ;;
            *)
                echo "Unknown option: $1"
                exit 1
                ;;
        esac
    done

    if [[ "$skip_wifi" == false ]]; then
        echo "[NET] Connecting to $TIAGO_SSID ..."
        nmcli device wifi connect $TIAGO_SSID password $TIAGO_PWD
    else
        echo "[NET] Skipping WiFi connection."
    fi

    export ROS_MASTER_URI="http://10.68.0.1:11311"
    export ROS_IP=$(hostname -I | cut -d' ' -f1)
    echo "[TIAGO] Connected to ROS master"
}

tiago-disconnect () {
    export ROS_MASTER_URI="http://localhost:11311"
    unset ROS_IP
    echo "[TIAGO] Disconnected from ROS master"
}

alias tc='tiago-connect'
alias td='tiago-disconnect'

alias rviz='rosrun rviz rviz' # -d $(rospack find tiago_rviz)/rviz/pge-nav.rviz'
alias gazebo='tiago-disconnect; roslaunch pmb2_gazebo pmb2_gazebo.launch'