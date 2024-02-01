


cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find packages
set(PACKAGES
  ament_cmake
  rclcpp
  std_msgs
  cv_bridge
)
foreach(PACKAGE ${PACKAGES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_executable(yolo src/YOLOv5.cpp src/CameraYolo.cpp src/main.cpp)
ament_target_dependencies(yolo rclcpp std_msgs cv_bridge)


# include directories
target_include_directories(yolo PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )


install(TARGETS
  yolo
  DESTINATION lib/${PROJECT_NAME}
  )





ament_package()