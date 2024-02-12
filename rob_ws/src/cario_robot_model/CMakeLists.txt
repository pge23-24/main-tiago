cmake_minimum_required(VERSION 3.0.2)
project(cario_robot_model)

find_package(
  catkin REQUIRED
  COMPONENTS cario_bringup
             cario_description)

include_directories(${catkin_INCLUDE_DIRS})
catkin_package()
