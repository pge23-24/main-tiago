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
  visualization_msgs
  multi_obstacles_tracker_msgs
  geometry_msgs
  
  tf
  
  laser_geometry

  message_filters

  pcl_ros
  pcl_conversions

  kalman_ros
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
add_executable(obstacles_cluster src/obstacles_cluster.cpp)
add_executable(obstacles_tracker src/obstacles_tracker.cpp)
add_executable(cluster_viz src/cluster_viz.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(scan_filter ${catkin_LIBRARIES})
target_link_libraries(obstacles_cluster ${catkin_LIBRARIES})
target_link_libraries(obstacles_tracker ${catkin_LIBRARIES})
target_link_libraries(cluster_viz ${catkin_LIBRARIES})