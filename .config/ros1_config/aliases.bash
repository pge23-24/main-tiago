# Connection to robot

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
                return
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

# RVIZ and Gazebo

alias rviz='roslaunch cario_rviz navigation.launch'
alias gazebo='tiago-disconnect; roslaunch pmb2_gazebo pmb2_gazebo.launch'

# Mapping

set-nav-mode () {
    if [[ "$1" != "MAP" && "$1" != "LOC" ]]; then
        echo "Invalid mode: $1"
        return
    fi

    rosservice call /pal_navigation_sm "input: '$1'"
}

start-mapping () {
    set-nav-mode MAP
    rviz
}

save-map () {
    # Get the map name as user input
    echo "Enter map name:"
    read map_name

    # Save the map
    rosrun map_server map_saver -f $map_name
    rosservice call /pal_map_manager/save_map "directory: '$map_name'"

    # Set the navigation mode to localization
    set-nav-mode LOC

    # Find a way to save the map locally
}

set-map () {
    rosservice call /pal_map_manager/change_map "input: '$1'"
}

alias snm='set-nav-mode'

# Package compilation

compile-cario-packages () {
    catkin build cario* --no-deps
    source /pal_mobile_base_ws/devel/setup.bash
}

alias cc="compile-cario-packages"
