cmake_minimum_required(VERSION 3.13)
project(cario_tools_testing)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
cmake_policy(SET CMP0079 NEW)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})




#############
## Testing ##
#############
find_package(rostest REQUIRED)


## Add gtest based cpp test target and link libraries

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(error_loc-test error_loc_test.cpp)

    target_link_libraries(error_loc-test ${PARENT_PROJECT_NAME}_loc_error_lib ${catkin_LIBRARIES})

endif()

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)