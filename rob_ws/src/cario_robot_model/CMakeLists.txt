cmake_minimum_required(VERSION 3.0.2)
project(cario_robot_model)

find_package(
  catkin REQUIRED
  COMPONENTS gazebo_plugins
             gazebo_ros
             pal_gazebo_plugins
             pal_gazebo_worlds
             pal_hardware_gazebo
             pmb2_bringup
             pmb2_controller_configuration_gazebo
             pmb2_description
             simple_models_gazebo)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package()
