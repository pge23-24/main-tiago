# ECAL to ROS2 Gateway

1. Run the CES Framework to acquire images and send them to ecal:

    Into a terminal, run the following command:

    ```bash
    sudo ~/Desktop/Fwk_licensed/CES_QtFramework_Application
    ```

2. Run the gateway eCAL to ROS2:

    Into a new terminal, run the following commands:

    ```bash
    cd ~/Desktop/CES_FWK_Gateway_ecalToRos2/CES_FWK_GATEWAY_ECALTOROS2/
    . ./install/setup.bash 
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 run ces_fwk_gateway_ecaltoros2 ces_fwk_gateway_ecaltoros2 install/ces_fwk_gateway_ecaltoros2/Config/config.cfg
    ```

3. Visualize images into rviz (an example is already configured)
    Into a new terminal, run the following commands:

    ```bash
    . /opt/ros/foxy/setup.bash
    ros2 run rviz2 rviz2
    ```

    To build the gateway if needed:
    Into a new terminal, run the following commands:

    ```bash
    . /opt/ros/foxy/setup.bash
    cd ~/Desktop/CES_FWK_Gateway_ecalToRos2/CES_FWK_GATEWAY_ECALTOROS2/
    colcon build
    ```
