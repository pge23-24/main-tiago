cmake_minimum_required(VERSION 3.0.2)
project(spot_control_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Add cmake target dependencies of the library as an example, code may need to
# be generated before libraries either from message generation or dynamic
# reconfigure
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide
add_executable(${PROJECT_NAME}_node src/spot_control_node.cpp)

# ##############################################################################
# Dependencies ##
# ##############################################################################

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ##############################################################################
# Librairies ##
# ##############################################################################

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

# ##############################################################################
# Sub Dirs ##
# ##############################################################################
set(PARENT_PROJECT_NAME ${PROJECT_NAME})
add_subdirectory(test)
