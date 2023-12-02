cmake_minimum_required(VERSION 3.0.2)
project(multi_obstacles_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  multi_obstacles_tracker_msgs
  tf
  geometry_msgs
  laser_geometry
  message_filters
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(scan_filter src/scan_filter.cpp)
add_executable(obstacles_clustering src/obstacles_clustering.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(scan_filter ${catkin_LIBRARIES})
target_link_libraries(obstacles_clustering ${catkin_LIBRARIES})