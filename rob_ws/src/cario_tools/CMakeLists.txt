cmake_minimum_required(VERSION 3.0.2)
project(cario_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs gazebo_msgs
                                        message_generation)

# Generate messages in the 'msg' folder
add_message_files(FILES ModelStatesTimeStamped.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs gazebo_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  message_runtime)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME}_loc_error_lib src/loc_error_node.cpp)

add_library(${PROJECT_NAME}_gazebo_synchro_lib src/gazebo_synchronizer.cpp)

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure
add_dependencies(${PROJECT_NAME}_loc_error_lib
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_gazebo_synchro_lib
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide
add_executable(loc_error src/loc_error_node.cpp)
add_executable(gazebo_synchro src/gazebo_synchronizer.cpp)

# ##############################################################################
# Dependencies ##
# ##############################################################################

add_dependencies(loc_error ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
add_dependencies(gazebo_synchro ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# ##############################################################################
# Librairies ##
# ##############################################################################

# Specify libraries to link a library or executable target against
target_link_libraries(loc_error ${catkin_LIBRARIES})

target_link_libraries(gazebo_synchro ${catkin_LIBRARIES})

# ##############################################################################
# Sub Dirs ##
# ##############################################################################
set(PARENT_PROJECT_NAME ${PROJECT_NAME})
add_subdirectory(test)
