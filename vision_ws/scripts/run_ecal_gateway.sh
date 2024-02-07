sudo ../Fwk_licensed/CES_QtFramework_Application &

cd /CES_FWK_Gateway_ecalToRos2/CES_FWK_GATEWAY_ECALTOROS2/
. ./install/setup.bash 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run ces_fwk_gateway_ecaltoros2 ces_fwk_gateway_ecaltoros2 install/ces_fwk_gateway_ecaltoros2/Config/config.cfg