cmake_minimum_required(VERSION 3.0.2)
project(cario_ihm_agent)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(NODE_NAME relay_controller)

find_package(catkin REQUIRED COMPONENTS 
roscpp
geometry_msgs
nav_msgs
rospy
std_msgs
multi_obstacles_tracker_msgs
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${NODE_NAME} src/robot_spot.cpp)

# Move add_dependencies after add_executable
add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})