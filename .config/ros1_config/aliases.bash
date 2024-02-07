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
alias simu='tiago-disconnect; roslaunch pmb2_2dnav_gazebo pmb2_navigation.launch public_sim:=true'

# Robot speed

alias get-robot-max-speed="rosrun dynamic_reconfigure dynparam get /move_base/PalLocalPlanner"
alias set-robot-max-speed="rosrun dynamic_reconfigure dynparam set /move_base/PalLocalPlanner max_vel_x"

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
    echo -n "Enter map name: "
    read map_name

    # Save the map
    echo "[NAV] Saving map on robot as \"$map_name\""
    rosrun map_server map_saver -f $map_name
    rosservice call /pal_map_manager/save_map "directory: '$map_name'"

    # Find a way to save the map locally
    echo -n "Save map locally? [y/n]: "
    read save_locally

    if [[ "$save_locally" == "y" ]]; then
        save-map-local $map_name
    fi

    # Set the navigation mode to localization
    echo "[NAV] Switching back to localization mode"
    set-nav-mode LOC
}

save-map-local () {
    map_name=$1

    echo "[NAV] Saving map \"$map_name\" locally"

    default_dir=/pal_mobile_base_ws/maps
    echo -n "Where should it be saved? [default: $default_dir]: "
    read save_dir

    if [[ "$save_dir" == "" ]]; then
        save_dir=$default_dir
    fi

    # resolve full path
    # save_dir =

    mkdir -p $save_dir
    scp -r pal@pmb2-46c:/home/pal/.pal/pmb2_maps/configurations/$map_name $save_dir/

    echo "[NAV] Map saved locally as \"$save_dir/$map_name\""
}

load-map () {
    path_to_map=$1
    # check if the path is valid
    if [[ ! -d "$path_to_map" ]]; then
        echo "Invalid path: $path_to_map"
        return
    fi

    # Get the map name as the last part of the path
    map_name=$(basename $path_to_map)

    scp -r $path_to_map pal@pmb2-46c:/home/pal/.pal/pmb2_maps/configurations/$map_name
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

compile-all-packages () {
    catkin build
    source /pal_mobile_base_ws/devel/setup.bash
}

alias cc="compile-cario-packages"
alias ca="compile-all-packages"


# Documentation

cario-help () {
clear
cat << EOF
Cario commands:
================

tiago-connect, tc
    Connect to the robot

tiago-disconnect, td
    disconnect from the robot

---

compile-cario-packages, cc
    Compile the cario packages

compile-all-packages, ca
    Compile all packages

---

rviz
    Launch RVIZ with the cario configuration

gazebo
    Launch Gazebo with the cario configuration

simu
    Launch base simulation used for testing

---

get-robot-max-speed
    Get the maximum speed of the robot

set-robot-max-speed <speed>
    Set the maximum speed of the robot

---

set-nav-mode <mode>, snm <mode>
    Set the navigation mode to either MAP or LOC

start-mapping
    Start mapping the environment

save-map
    Save the map on the robot and optionally locally

save-map-local <map_name>
    Save the map locally

load-map <path_to_map>
    Load a map from a local directory onto the robot

set-map <map_name>
    Set the map to be used for localization

EOF
}
