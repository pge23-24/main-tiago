cmake_minimum_required(VERSION 3.0.2)
project(cario_map)

find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sensor_msgs
                                        message_generation)

add_service_files(FILES GetMapInfo.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp std_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(get_map src/get_map.cpp)
target_include_directories(get_map PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(get_map ${catkin_LIBRARIES} yaml-cpp::yaml-cpp)
add_dependencies(get_map ${catkin_EXPORTED_TARGETS})

# ##############################################################################
# yaml-cpp install start
# ##############################################################################

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0 # Replace with the desired version or tag
)

FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()
