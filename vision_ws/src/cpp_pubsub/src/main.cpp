#ifndef CLASSICAL
#define CLASSICAL
#include <memory>
#include <iostream>
#include <fstream>
#endif

#ifndef ROS
#define ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#endif

#include "../include/CameraYolo.hpp"

using namespace cv;
using namespace std;

int main(int argc, char * argv[])
{
  cout << CV_VERSION << endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraYolo>());
  rclcpp::shutdown();
  return 0;
}