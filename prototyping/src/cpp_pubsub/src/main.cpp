#ifndef CLASSICAL
#define CLASSICAL
#include <memory>
#include <iostream>
#include <fstream>
#endif

#ifndef ROS
#define ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#endif

#include "../include/CameraYolo.hpp"

int main(int argc, char * argv[])
{
  std::cout << cv::CV_VERSION << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraYolo>());
  rclcpp::shutdown();
  return 0;
}