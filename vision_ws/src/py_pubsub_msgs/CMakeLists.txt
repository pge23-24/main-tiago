cmake_minimum_required(VERSION 3.5)
project(py_pubsub_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


ament_export_dependencies(rosidl_default_runtime)
ament_package()

# If you have C++ source code, add the following lines to build and install
# Include C++ headers
#include_directories(include)

# Declare a C++ library (if you have any C++ code for your package)
# add_library(${PROJECT_NAME}_library
#   src/my_library_source.cpp
# )

# Add cmake target dependencies of the library
# (if the library depends on any other libraries)
# add_dependencies(${PROJECT_NAME}_library
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${ament_EXPORTED_TARGETS}
# )

# Declare a C++ executable (if you have C++ executables in your package)
# add_executable(${PROJECT_NAME}_node src/my_node_source.cpp)

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}_library
# )

# Install C++ executables
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install C++ libraries
# install(TARGETS ${PROJECT_NAME}_library
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# Install C++ header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
# )

# Install other files like launch files etc.
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )